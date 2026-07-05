"""
Fitness Tracker - CircuitPython
Hardware: SuperMini NRF52840, MAX30102, LSM6DS3, TMP117
Features: HR, SpO2, HRV, Steps, Skin Temp, Sleep Detection, CSV Logging -> BLE UART

Wiring:
  MAX30102  -> 3V3, GND, SDA, SCL
  LSM6DS3   -> 3V3, GND, SDA, SCL
  TMP117    -> 3V3, GND, SDA, SCL
  Pull-ups: 10k ohm from 3V3 to SDA and SCL

Libraries needed in /lib:
  adafruit_ble/
  adafruit_lsm6ds/
  adafruit_register/
  adafruit_tmp117.mpy

BLE protocol:
  On connect, phone sends: SET_TIME:<unix_timestamp>
  Board streams CSV log when phone sends: GET_LOG
  Board clears log when phone sends: CLEAR_LOG
  Normal data: HR:<n>,SpO2:<n>,HRV:<n>,Steps:<n> every 2s

State machine:
  awake       -> full sample rate, BLE on, streaming
      |
      |-- sleep window + still 5 min --> sleep
      |-- not sleep window + still 10 min --> unattended
      |
  sleep       -> full sample rate, BLE OFF, logging only
      |
      |-- movement + outside sleep window --> awake
      |
  unattended  -> HR only every 5 min, BLE OFF, no logging
      |
      |-- movement confirmed for 2 min --> awake

Power profile (approx):
  awake:       25-35mA  (BLE on, all sensors)
  sleep:        8-12mA  (BLE off, all sensors)
  unattended:   3-5mA   (BLE off, HR duty cycled)
"""

import time
import math
import board
import busio
import digitalio
from adafruit_ble import BLERadio
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.nordic import UARTService
from adafruit_lsm6ds.lsm6ds33 import LSM6DS33 as LSM6DS3
from adafruit_lsm6ds import Rate, AccelRange
import supervisor
import microcontroller
import adafruit_tmp117
import analogio

# Capture USB state at boot for disconnect detection
_usb_was_connected = supervisor.runtime.usb_connected

# Battery voltage monitoring
# Multiplier of 40.41 validated against multimeter reading for this specific board
BAT_MULTIPLIER = 40.41
_bat_pin = analogio.AnalogIn(board.BAT_VOLT)
_bat_smooth = 0.0  # smoothed battery voltage

LOG_FILE = "/data.csv"
BOOT_LOG_FILE = "/boot_log.txt"


def _boot_log(msg):
    """Append a boot-stage milestone to a file so progress can be inspected
    after a battery-only run with no serial console attached."""
    try:
        with open(BOOT_LOG_FILE, "a") as f:
            f.write("{} | usb={}\n".format(msg, supervisor.runtime.usb_connected))
    except OSError as e:
        print("boot log write failed:", e)


def _i2c_bus_recovery():
    """Manually clock SCL to free a slave that's stuck holding SDA low.
    On battery, MAX30102 (powered from RAW) can power up out of sync with
    the IMU/TMP117 (powered from 3V3), leaving it mid-transaction with SDA
    latched low — which hangs busio.I2C() before it can even finish init.
    This bit-bangs up to 9 clock pulses (the max a stuck I2C slave can be
    waiting on) and issues a STOP condition to force the bus free."""
    scl = digitalio.DigitalInOut(board.SCL)
    sda = digitalio.DigitalInOut(board.SDA)
    scl.direction = digitalio.Direction.OUTPUT
    sda.direction = digitalio.Direction.INPUT
    scl.value = True

    for _ in range(9):
        if sda.value:
            break
        scl.value = False
        time.sleep(0.00005)
        scl.value = True
        time.sleep(0.00005)

    # Issue a STOP condition: SDA low-to-high while SCL is high
    sda.direction = digitalio.Direction.OUTPUT
    sda.value = False
    time.sleep(0.00005)
    scl.value = True
    time.sleep(0.00005)
    sda.value = True
    time.sleep(0.00005)

    scl.deinit()
    sda.deinit()

# ─────────────────────────────────────────────
#  MAX30102 Driver
# ─────────────────────────────────────────────

MAX30102_ADDR   = 0x57
REG_FIFO_WR_PTR = 0x04
REG_OVF_COUNTER = 0x05
REG_FIFO_RD_PTR = 0x06
REG_FIFO_DATA   = 0x07
REG_FIFO_CONFIG = 0x08
REG_MODE_CONFIG = 0x09
REG_SPO2_CONFIG = 0x0A
REG_LED1_PA     = 0x0C
REG_LED2_PA     = 0x0D

class MAX30102:
    def __init__(self, i2c):
        self.i2c = i2c
        self._setup()

    def _write(self, reg, value):
        while not self.i2c.try_lock():
            pass
        try:
            self.i2c.writeto(MAX30102_ADDR, bytes([reg, value]))
        finally:
            self.i2c.unlock()

    def _read(self, reg, length=1):
        while not self.i2c.try_lock():
            pass
        try:
            buf = bytearray(length)
            self.i2c.writeto_then_readfrom(MAX30102_ADDR, bytes([reg]), buf)
        finally:
            self.i2c.unlock()
        return buf

    def _setup(self):
        self._write(REG_MODE_CONFIG, 0x40)
        time.sleep(0.1)
        self._write(REG_MODE_CONFIG, 0x03)
        self._write(REG_SPO2_CONFIG, 0x27)
        self._write(REG_FIFO_CONFIG, 0x1F)  # no averaging, rollover enabled
        self._write(REG_LED1_PA,     0x3F)  # RED LED — validated for wrist use
        self._write(REG_LED2_PA,     0x3F)  # IR LED — validated for wrist use
        self._write(REG_FIFO_WR_PTR, 0x00)
        self._write(REG_OVF_COUNTER, 0x00)
        self._write(REG_FIFO_RD_PTR, 0x00)

    def shutdown(self):
        """Put MAX30102 into low power shutdown mode."""
        self._write(REG_MODE_CONFIG, 0x80)

    def wakeup(self):
        """Wake MAX30102 from shutdown and restore HR+SpO2 mode."""
        self._write(REG_MODE_CONFIG, 0x03)

    def read_fifo_sample(self):
        wr = self._read(REG_FIFO_WR_PTR)[0]
        rd = self._read(REG_FIFO_RD_PTR)[0]
        if wr == rd:
            return None
        data = self._read(REG_FIFO_DATA, 6)
        red = ((data[0] & 0x03) << 16) | (data[1] << 8) | data[2]
        ir  = ((data[3] & 0x03) << 16) | (data[4] << 8) | data[5]
        return red, ir


# ─────────────────────────────────────────────
#  Heart Rate & HRV Processor
#
#  Pipeline:
#  1. IIR bandpass filter isolates heartbeat signal
#     from DC baseline and high-frequency noise
#  2. Zero-crossing derivative peak detection
#  3. Wall-clock RR interval measurement
#  4. Outlier rejection + RMSSD computation
#
#  Key values validated via shell testing:
#    LP_ALPHA 0.80 — best HR accuracy vs noise
#    REFRACTORY_MS 700 — cleanly skips dicrotic notch
#    IR_THRESHOLD 10000 — separates finger-on (~130k)
#                         from finger-off (~1600)
# ─────────────────────────────────────────────

class HRProcessor:
    SAMPLE_RATE_HZ  = 100
    MS_PER_SAMPLE   = 10
    REFRACTORY_MS   = 700      # validated — skips dicrotic notch (~400ms after peak)
    SIGNAL_LOSS_S   = 3.0
    IR_THRESHOLD    = 10000    # finger-off ~1600, finger-on ~130000
    RR_OUTLIER_PCT  = 0.25     # reject RR > 25% from recent mean
    RR_WARMUP       = 4        # discard first N intervals while amplitude buffer settles

    # IIR filter coefficients (validated via shell test)
    HP_ALPHA        = 0.95     # high-pass: removes DC baseline
    LP_ALPHA        = 0.80     # low-pass: removes noise, preserves heartbeat

    ACQUIRE_SAMPS   = 200      # samples to let filter settle after finger placement

    def __init__(self):
        self._sample_count  = 0
        self._rr_list       = []
        self.bpm            = 0
        self.rmssd          = 0
        self._last_signal   = time.monotonic()
        self._acquire_count = 0

        # Filter state
        self._hp_prev_raw   = 0.0
        self._hp_prev_out   = 0.0
        self._lp_prev       = 0.0

        # Peak detection state
        self._last_peak_ms  = 0      # wall-clock ms of last peak (RR timing)
        self._rr_warmup     = 0      # warmup interval counter
        self._prev_filtered = 0.0
        self._prev_deriv    = 0.0
        self._peak_amp_buf  = []     # recent peak amplitudes for adaptive threshold

    def _filter(self, raw):
        """Two-stage IIR bandpass filter."""
        hp = self.HP_ALPHA * (self._hp_prev_out + raw - self._hp_prev_raw)
        self._hp_prev_raw = raw
        self._hp_prev_out = hp
        lp = (1.0 - self.LP_ALPHA) * hp + self.LP_ALPHA * self._lp_prev
        self._lp_prev = lp
        return lp

    def update(self, ir):
        """Call once per sample."""
        self._sample_count += 1

        if ir >= self.IR_THRESHOLD:
            self._last_signal = time.monotonic()
            self._acquire_count += 1
        else:
            if time.monotonic() - self._last_signal > self.SIGNAL_LOSS_S:
                self._reset()
            elif time.monotonic() - self._last_signal > 0.5:
                # Signal dropping — reset acquire so filter re-settles on replacement
                self._acquire_count = 0
            return

        filtered = self._filter(float(ir))

        # Let filter settle after finger placement
        if self._acquire_count == self.ACQUIRE_SAMPS:
            self._peak_amp_buf = []   # clear amplitude buffer at end of acquisition
        if self._acquire_count < self.ACQUIRE_SAMPS:
            self._prev_filtered = filtered
            self._prev_deriv = 0.0
            return

        deriv = filtered - self._prev_filtered
        now_ms = time.monotonic_ns() // 1_000_000

        # Refractory check — wall-clock based, immune to burst read timing
        refractory_ok = (self._last_peak_ms == 0 or
                         (now_ms - self._last_peak_ms) > self.REFRACTORY_MS)

        if self._prev_deriv > 0 and deriv <= 0 and refractory_ok:
            peak_val = self._prev_filtered

            # Adaptive amplitude threshold — peak must exceed 30% of recent mean
            if self._peak_amp_buf:
                mean_amp = sum(self._peak_amp_buf) / len(self._peak_amp_buf)
                if peak_val < mean_amp * 0.30:
                    self._prev_filtered = filtered
                    self._prev_deriv = deriv
                    return

            if peak_val > 0:
                if self._last_peak_ms > 0:
                    rr_ms = now_ms - self._last_peak_ms

                    if 333 < rr_ms < 2000:
                        if self._rr_warmup < self.RR_WARMUP:
                            self._rr_warmup += 1
                        else:
                            accept = True
                            if len(self._rr_list) >= 4:
                                mean_rr = sum(self._rr_list) / len(self._rr_list)
                                if abs(rr_ms - mean_rr) > mean_rr * self.RR_OUTLIER_PCT:
                                    accept = False
                            if accept:
                                self._rr_list.append(rr_ms)
                                if len(self._rr_list) > 12:
                                    self._rr_list.pop(0)
                                self._compute()

                self._peak_amp_buf.append(abs(peak_val))
                if len(self._peak_amp_buf) > 8:
                    self._peak_amp_buf.pop(0)
                self._last_peak_ms = now_ms

        self._prev_filtered = filtered
        self._prev_deriv = deriv

    def _reset(self):
        self.bpm            = 0
        self.rmssd          = 0
        self._rr_list       = []
        self._acquire_count = 0
        self._hp_prev_raw   = 0.0
        self._hp_prev_out   = 0.0
        self._lp_prev       = 0.0
        self._last_peak_ms  = 0
        self._rr_warmup     = 0
        self._prev_filtered = 0.0
        self._prev_deriv    = 0.0
        self._peak_amp_buf  = []

    def _compute(self):
        if len(self._rr_list) < 4:
            return
        mean_rr = sum(self._rr_list) / len(self._rr_list)
        if mean_rr <= 0:
            return
        bpm = round(60000 / mean_rr)
        if not (30 <= bpm <= 180):
            return
        self.bpm = bpm
        sq_diffs = [(self._rr_list[i+1] - self._rr_list[i])**2
                    for i in range(len(self._rr_list)-1)]
        self.rmssd = round((sum(sq_diffs) / len(sq_diffs)) ** 0.5)


# ─────────────────────────────────────────────
#  SpO2 Processor
# ─────────────────────────────────────────────

class SpO2Processor:
    WINDOW    = 200   # larger window for stable R ratio
    MIN_AC_IR = 50    # minimum IR AC amplitude for valid reading
    SMOOTH    = 0.3   # smoothing factor — prevents single-sample jumps

    def __init__(self):
        self._red_buf   = []
        self._ir_buf    = []
        self.spo2       = 0
        self._prev_spo2 = 0

    def update(self, red, ir):
        self._red_buf.append(red)
        self._ir_buf.append(ir)
        if len(self._red_buf) > self.WINDOW:
            self._red_buf.pop(0)
            self._ir_buf.pop(0)
        if len(self._red_buf) == self.WINDOW:
            self._compute()

    def _compute(self):
        red_dc = sum(self._red_buf) / self.WINDOW
        ir_dc  = sum(self._ir_buf)  / self.WINDOW
        if ir_dc == 0 or red_dc == 0:
            return
        red_ac = max(self._red_buf) - min(self._red_buf)
        ir_ac  = max(self._ir_buf)  - min(self._ir_buf)
        if ir_ac < self.MIN_AC_IR or red_ac == 0:
            return
        R = (red_ac / red_dc) / (ir_ac / ir_dc)
        if not (0.4 <= R <= 1.2):
            return
        spo2 = -45.060 * R * R + 30.354 * R + 94.845
        spo2 = max(70, min(100, round(spo2)))
        if self._prev_spo2 == 0:
            self._prev_spo2 = spo2
        smoothed = round(self._prev_spo2 * (1 - self.SMOOTH) + spo2 * self.SMOOTH)
        self._prev_spo2 = smoothed
        self.spo2 = smoothed


# ─────────────────────────────────────────────
#  Movement Tracker
# ─────────────────────────────────────────────

class MovementTracker:
    GRAVITY        = 9.8
    WINDOW_SECS    = 10
    SAMPLE_RATE    = 10
    WINDOW_SIZE    = WINDOW_SECS * SAMPLE_RATE
    BASELINE_SIZE  = 300

    def __init__(self):
        self._mag_buf     = []
        self.score        = 0.0
        self._day_scores  = []
        self.day_baseline = 0.5

    def update(self, ax, ay, az, is_daytime):
        mag = math.sqrt(ax*ax + ay*ay + az*az)
        dynamic = abs(mag - self.GRAVITY)
        self._mag_buf.append(dynamic)
        if len(self._mag_buf) > self.WINDOW_SIZE:
            self._mag_buf.pop(0)
        if len(self._mag_buf) >= 2:
            mean = sum(self._mag_buf) / len(self._mag_buf)
            variance = sum((x - mean)**2 for x in self._mag_buf) / len(self._mag_buf)
            self.score = math.sqrt(variance)
        if is_daytime and self.score > 0:
            self._day_scores.append(self.score)
            if len(self._day_scores) > self.BASELINE_SIZE:
                self._day_scores.pop(0)
            if len(self._day_scores) >= 10:
                self.day_baseline = sum(self._day_scores) / len(self._day_scores)

    @property
    def is_still(self):
        return self.score < (self.day_baseline * 0.2)

    @property
    def is_active(self):
        return self.score > (self.day_baseline * 0.35)


# ─────────────────────────────────────────────
#  Sleep & Unattended State Machine
# ─────────────────────────────────────────────

class SleepTracker:
    AWAKE      = "awake"
    LIGHT      = "light"
    DEEP       = "deep"
    REM        = "rem"
    UNATTENDED = "unattended"

    SLEEP_ONSET_MINS      = 5
    UNATTENDED_ONSET_MINS = 10
    WAKE_CONFIRM_MINS     = 2

    def __init__(self):
        self.state           = self.AWAKE
        self._still_minutes  = 0
        self._active_minutes = 0
        self._sleep_onset_hr = None

    def update(self, hour, is_still, is_active, bpm, rmssd):
        is_sleep_window = (hour >= 21 or hour < 9)

        if self.state == self.AWAKE:
            if is_still:
                self._still_minutes += (1 / 60.0)
                if is_sleep_window and self._still_minutes >= self.SLEEP_ONSET_MINS:
                    self.state = self.LIGHT
                    self._sleep_onset_hr = bpm if bpm > 0 else 60
                    self._still_minutes = 0
                elif not is_sleep_window and self._still_minutes >= self.UNATTENDED_ONSET_MINS:
                    self.state = self.UNATTENDED
                    self._still_minutes = 0
            else:
                self._still_minutes = 0

        elif self.state in (self.LIGHT, self.DEEP, self.REM):
            if not is_still and not is_sleep_window:
                self.state = self.AWAKE
                self._sleep_onset_hr = None
                self._still_minutes = 0
                return self.state
            if bpm > 0 and rmssd > 0 and self._sleep_onset_hr:
                hr_ratio = bpm / self._sleep_onset_hr
                if hr_ratio < 0.88 and rmssd > 40:
                    self.state = self.DEEP
                elif hr_ratio > 0.95 and rmssd < 25:
                    self.state = self.REM
                else:
                    self.state = self.LIGHT
            elif is_still:
                self.state = self.LIGHT

        elif self.state == self.UNATTENDED:
            if is_active:
                self._active_minutes += (1 / 60.0)
                if self._active_minutes >= self.WAKE_CONFIRM_MINS:
                    self.state = self.AWAKE
                    self._active_minutes = 0
                    self._still_minutes = 0
            else:
                self._active_minutes = 0

        return self.state

    @property
    def is_sleep(self):
        return self.state in (self.LIGHT, self.DEEP, self.REM)

    @property
    def is_unattended(self):
        return self.state == self.UNATTENDED

    @property
    def ble_should_be_off(self):
        return self.state in (self.LIGHT, self.DEEP, self.REM, self.UNATTENDED)

    @property
    def log_state_str(self):
        return self.state


# ─────────────────────────────────────────────
#  Data Logger
# ─────────────────────────────────────────────

class DataLogger:
    FLUSH_INTERVAL = 60
    MAX_BUF        = 120   # cap buffers to limit RAM usage

    def __init__(self):
        self._hr_buf     = []
        self._spo2_buf   = []
        self._hrv_buf    = []
        self._mov_buf    = []
        self._last_flush = time.monotonic()
        self._ensure_header()

    def _ensure_header(self):
        try:
            with open(LOG_FILE, "r") as f:
                if f.read(3) == "ts,":
                    return
        except OSError:
            pass
        with open(LOG_FILE, "w") as f:
            f.write("ts,hr,spo2,hrv,steps,mov,temp,bat,state\n")

    def add_sample(self, hr, spo2, hrv, movement_score):
        if hr > 0:
            self._hr_buf.append(hr)
            if len(self._hr_buf) > self.MAX_BUF:
                self._hr_buf.pop(0)
        if spo2 > 0:
            self._spo2_buf.append(spo2)
            if len(self._spo2_buf) > self.MAX_BUF:
                self._spo2_buf.pop(0)
        if hrv > 0:
            self._hrv_buf.append(hrv)
            if len(self._hrv_buf) > self.MAX_BUF:
                self._hrv_buf.pop(0)
        self._mov_buf.append(movement_score)
        if len(self._mov_buf) > self.MAX_BUF:
            self._mov_buf.pop(0)

    def _median(self, buf):
        if not buf:
            return 0
        s = sorted(buf)
        mid = len(s) // 2
        return s[mid] if len(s) % 2 else (s[mid-1] + s[mid]) // 2

    def flush(self, unix_ts, steps, sleep_state, skin_temp=0.0, bat_v=0.0):
        now = time.monotonic()
        if now - self._last_flush < self.FLUSH_INTERVAL:
            return False
        hr   = self._median(self._hr_buf)
        spo2 = self._median(self._spo2_buf)
        hrv  = self._median(self._hrv_buf)
        mov  = round(sum(self._mov_buf) / len(self._mov_buf), 4) if self._mov_buf else 0
        try:
            with open(LOG_FILE, "a") as f:
                f.write("{},{},{},{},{},{},{},{},{}\n".format(
                    unix_ts, hr, spo2, hrv, steps, mov, skin_temp, bat_v, sleep_state))
        except OSError as e:
            print("Log write error:", e)
        self._hr_buf.clear()
        self._spo2_buf.clear()
        self._hrv_buf.clear()
        self._mov_buf.clear()
        self._last_flush = now
        return True

    def stream_log(self, uart):
        try:
            with open(LOG_FILE, "r") as f:
                while True:
                    chunk = f.read(100)
                    if not chunk:
                        break
                    uart.write(chunk.encode())
                    time.sleep(0.05)
            uart.write(b"END_LOG\n")
        except OSError:
            uart.write(b"NO_LOG\n")

    def clear_log(self):
        with open(LOG_FILE, "w") as f:
            f.write("ts,hr,spo2,hrv,steps,mov,temp,bat,state\n")


# ─────────────────────────────────────────────
#  Time Manager
# ─────────────────────────────────────────────

class TimeManager:
    def __init__(self):
        self._offset = 0
        self._synced = False

    def sync(self, unix_ts):
        self._offset = unix_ts - time.monotonic()
        self._synced = True
        print("Time synced. Unix ts:", unix_ts)

    @property
    def unix(self):
        return int(time.monotonic() + self._offset)

    @property
    def hour(self):
        if not self._synced:
            return 12
        return (self.unix % 86400) // 3600

    @property
    def is_daytime(self):
        return 6 <= self.hour < 21


# ─────────────────────────────────────────────
#  BLE Manager
# ─────────────────────────────────────────────

class BLEManager:
    def __init__(self, ble, advertisement):
        self._ble          = ble
        self._advertisement = advertisement
        self._advertising  = False

    def start(self):
        if not self._advertising and not self._ble.connected:
            try:
                self._ble.start_advertising(self._advertisement)
                self._advertising = True
                print("BLE advertising started")
                _boot_log("BLE advertising STARTED")
            except Exception as e:
                print("BLE advertising FAILED to start:", e)
                _boot_log("BLE advertising FAILED: {}".format(e))

    def stop(self):
        if self._advertising:
            self._ble.stop_advertising()
            self._advertising = False
            print("BLE advertising stopped")

    @property
    def connected(self):
        return self._ble.connected


# ─────────────────────────────────────────────
#  Main Setup
# ─────────────────────────────────────────────

print("Initializing I2C...")
_boot_log("==== BOOT START ====")
time.sleep(3.0)  # allow all sensors to fully settle on cold boot
_boot_log("starting I2C bus recovery")
_i2c_bus_recovery()
_boot_log("I2C bus recovery done")
i2c = busio.I2C(board.SCL, board.SDA, frequency=50000)
_boot_log("I2C bus OK")

print("Initializing MAX30102...")
max30102 = None
for attempt in range(10):
    try:
        max30102 = MAX30102(i2c)
        print("MAX30102 ready on attempt", attempt + 1)
        _boot_log("MAX30102 OK on attempt {}".format(attempt + 1))
        break
    except Exception as e:
        print("MAX30102 attempt", attempt + 1, "failed:", e)
        _boot_log("MAX30102 attempt {} failed: {}".format(attempt + 1, e))
        time.sleep(1.0)
if max30102 is None:
    print("MAX30102 failed to initialize — halting")
    _boot_log("MAX30102 FAILED — halting (RuntimeError raised)")
    raise RuntimeError("MAX30102 not found")
time.sleep(0.5)

print("Initializing LSM6DS3...")
try:
    i2c.unlock()
except Exception:
    pass
try:
    imu = LSM6DS3(i2c, address=0x6B)
    imu.accelerometer_range = AccelRange.RANGE_2G
    imu.accelerometer_data_rate = Rate.RATE_26_HZ
    imu.gyro_data_rate = Rate.RATE_SHUTDOWN
    imu.pedometer_enable = True
    print("LSM6DS3 ready")
    _boot_log("IMU OK")
except Exception as e:
    print("LSM6DS3 failed to initialize — halting:", e)
    _boot_log("IMU FAILED — halting: {}".format(e))
    raise

print("Initializing BLE...")
time.sleep(1.0)  # allow radio to stabilize on battery boot
try:
    ble_radio  = BLERadio()
    ble_radio.name = "FitTracker"
    uart_service   = UARTService()
    advertisement  = ProvideServicesAdvertisement(uart_service)
    print("BLE radio ready")
    _boot_log("BLE radio OK")
except Exception as e:
    print("BLE radio failed to initialize — halting:", e)
    _boot_log("BLE radio FAILED — halting: {}".format(e))
    raise

print("Initializing TMP117...")
try:
    tmp117 = adafruit_tmp117.TMP117(i2c)
    tmp117.low_limit  = -40   # disable alert LED — prevents interference with MAX30102
    tmp117.high_limit = 150
    tmp117_ok = True
    print("TMP117 ready")
    _boot_log("TMP117 OK")
except Exception as e:
    print("TMP117 not found:", e)
    _boot_log("TMP117 FAILED (non-fatal): {}".format(e))
    tmp117 = None
    tmp117_ok = False

hr_proc       = HRProcessor()
spo2_proc     = SpO2Processor()
movement      = MovementTracker()
sleep_tracker = SleepTracker()
logger        = DataLogger()
clock         = TimeManager()
ble_mgr       = BLEManager(ble_radio, advertisement)

SEND_INTERVAL_S        = 2.0
last_send              = time.monotonic()
UNATTENDED_HR_INTERVAL = 300
last_unattended_hr     = time.monotonic()
_ble_was_off           = False


# ─────────────────────────────────────────────
#  Main Loop
# ─────────────────────────────────────────────

print("Starting FitTracker")
print("State: awake | BLE: on")

ble_mgr.start()

while True:

    now = time.monotonic()

    # ── Restart cleanly if USB was just unplugged ──
    # This ensures BLE advertising starts fresh on battery boot
    if _usb_was_connected and not supervisor.runtime.usb_connected:
        print("USB disconnected — restarting for clean battery boot")
        time.sleep(0.5)
        microcontroller.reset()

    # ── Read accelerometer (always) ──
    try:
        ax, ay, az = imu.acceleration
        movement.update(ax, ay, az, clock.is_daytime)
    except Exception:
        pass

    # ── Update state machine ──
    sleep_tracker.update(
        clock.hour,
        movement.is_still,
        movement.is_active,
        hr_proc.bpm,
        hr_proc.rmssd
    )

    current_state = sleep_tracker.state

    # ── BLE control based on state ──
    if sleep_tracker.ble_should_be_off:
        if not _ble_was_off:
            ble_mgr.stop()
            _ble_was_off = True
            print("State:", current_state, "| BLE: off")
    else:
        if _ble_was_off:
            ble_mgr.start()
            _ble_was_off = False
            print("State: awake | BLE: on")

    # ── UNATTENDED mode — duty cycle HR only every 5 min ──
    if sleep_tracker.is_unattended:
        if now - last_unattended_hr >= UNATTENDED_HR_INTERVAL:
            max30102.wakeup()
            time.sleep(0.5)
            for _ in range(20):
                sample = max30102.read_fifo_sample()
                if sample:
                    red, ir = sample
                    hr_proc.update(ir)
                    if ir > HRProcessor.IR_THRESHOLD:
                        spo2_proc.update(red, ir)
                time.sleep(0.1)
            max30102.shutdown()
            last_unattended_hr = now
            print("Unattended HR check:", hr_proc.bpm, "bpm")
        time.sleep(1.0)
        continue

    # ── NORMAL and SLEEP modes — drain FIFO each iteration ──
    for _ in range(4):
        sample = max30102.read_fifo_sample()
        if sample is None:
            break
        red, ir = sample
        hr_proc.update(ir)
        if ir > HRProcessor.IR_THRESHOLD:
            spo2_proc.update(red, ir)

    # ── Read temperature ──
    skin_temp = 0.0
    if tmp117_ok:
        try:
            skin_temp = round(tmp117.temperature, 2)
        except Exception:
            pass

    # ── Read battery voltage (smoothed to reduce ADC noise) ──
    raw_bat = _bat_pin.value * 3.3 / 65536 * BAT_MULTIPLIER
    if _bat_smooth == 0.0:
        _bat_smooth = raw_bat
    _bat_smooth = _bat_smooth * 0.95 + raw_bat * 0.05
    bat_v = round(_bat_smooth, 2)

    # ── Log sample ──
    logger.add_sample(hr_proc.bpm, spo2_proc.spo2,
                      hr_proc.rmssd, movement.score)
    logger.flush(clock.unix, imu.pedometer_steps, current_state, skin_temp, bat_v)

    # ── BLE communication (awake only) ──
    if ble_mgr.connected:
        if uart_service.in_waiting:
            raw = uart_service.read(uart_service.in_waiting)
            if raw:
                msg = raw.decode("utf-8", "ignore").strip()
                if msg.startswith("SET_TIME:"):
                    try:
                        ts = int(msg.split(":")[1])
                        clock.sync(ts)
                        uart_service.write(b"TIME_OK\n")
                    except Exception:
                        uart_service.write(b"TIME_ERR\n")
                elif msg == "GET_LOG":
                    logger.stream_log(uart_service)
                elif msg == "CLEAR_LOG":
                    logger.clear_log()
                    uart_service.write(b"LOG_CLEARED\n")

        if now - last_send >= SEND_INTERVAL_S:
            line = "HR:{},SpO2:{},HRV:{},Steps:{},Mov:{:.3f},Temp:{:.2f},Bat:{:.2f},Sleep:{}\n".format(
                hr_proc.bpm, spo2_proc.spo2, hr_proc.rmssd,
                imu.pedometer_steps, movement.score, skin_temp, bat_v, current_state)
            try:
                uart_service.write(line.encode())
            except Exception:
                pass
            print(line, end="")
            last_send = now

    time.sleep(0.02)
