# sleep-tracker
Everything needed to create a sleep or fitness tracker at home

Overview:
The point of this project is to see how realistic it is to make a DIY sleep/fitness tracker with little to no relevant experience. A major aspect of this project is testing the abilities of AI. Claude was used to help define hardware, facilitate wiring and writing software.

Parts List:

Processor: NRF52840 https://www.amazon.com/Teyleten-Robot-Development-Bluetooth-Management/dp/B0CYLNZ6V4/ref=sr_1_1?crid=3FP25UKNPDTQH&dib=eyJ2IjoiMSJ9.sDJflg29f5DUcSLa7PGYuC7edLoVuob2TCNQWW-IEui6WC2FTkhG0hvcwbMVQoEd5RZfT1QAPAQfT20uYk1yp0uZDOL9st8hNa_W6nNzb08LrXXFAHIjFUrX8Z7vPoT_LmkivjytayP-YYoAgx7fm_R4jWnaFtLYPyR9_wgkw8_7LMryQQ4ymxvpOeqWmSX99NLqWScASm1u6uZu2RUmPbzSDCyp0ddZ-MnV5rUOyrQ.CdUlqNartXnFxaoyawTi1b6F0zIJPtHJoX9oD2af53I&dib_tag=se&keywords=teyleten+nrf52840&qid=1783271200&sprefix=teyleten+nrf52840%2Caps%2C123&sr=8-1

PPG sensor (HR, HRV, Sp02): MAX30102 I suggest buying multiple as I broke a few trying to assemble the device. https://www.amazon.com/Diitao-MAX30102-Detection-Concentration-Calculation/dp/B0B2R9WNVP/ref=sr_1_10?crid=2ZU56KVCSJ1A0&dib=eyJ2IjoiMSJ9.QwMpqViDtux0YmLojEFQNGK6ZgwegXtJ8jJIKjISCXAzxf7q3f0TUrZGP66UIQ52J3PV8DPmf_MIL7qg6VfJXuUfKdeZ0LA1JlFdsA0ImuqTpCsJyio_ojVYwagpT-InkVI1KaUkLERYTwCoX-bHCn5RoJo7QDBwwgED1YSn2wJVdof9Dgqim13dmnsrO880aF8d0vO7nnH9eGafZDUaY-J2Ld0a4KfugSInRYcciJw.vgOgDiS2jM-sfvwC1sNXUiY0sHcsZfuzFU7G1QFn2h0&dib_tag=se&keywords=max30102&qid=1783270568&sprefix=max30102%2Caps%2C153&sr=8-10&th=1

IMU (movement): LSM6DS3 https://www.amazon.com/LSM6DS3-Accelerometer-Gyroscope-Temperature-Interface/dp/B0FKT9ZR2X/ref=sr_1_1?crid=1GZBQNHXZ96BS&dib=eyJ2IjoiMSJ9.HoEK97beHGO-5O3xdNx_T2a_i0o-fpnLUbiTzKCeOMuHpumUDWwGJZTOh0GZGinmCMdUdOntKEZw0e5sm-crZ8k9bwmJv92lIjXq4WJHXB_rrA1osIjHSxyeC6O3lqYyfUG-2EacYnT2HdRN1yTfzObOLeEy9z85MguC1FwcQ7o85N01GcHm1ReKsq8YIvWIhdO9KmVGWR6q5tik6XlBZMc0XIQVU-3nrgIN6fBdRRo.rZGWpld0GjNai9g7lQwukak8JRGw2kWHhoVPGdYS3-Y&dib_tag=se&keywords=imu+sensor+lsm+6ds3&qid=1783270735&sprefix=imu+sensor+lsm+6ds3%2Caps%2C106&sr=8-1

Temperature: TMP117 The one I bought is no longer available on amazon. Here is the link to their direct site https://www.sparkfun.com/sparkfun-high-precision-temperature-sensor-tmp117-qwiic.html?utm_source=google&utm_medium=cpc&utm_campaign=XRP+Robotics+Kit+%7C+PMax&utm_content=First+Robotics+%7C+XRP-for+first&gad_source=1&gad_campaignid=21251727806&gbraid=0AAAAADsj4ESFoYykzT9mrHTJ1bKErB2GL&gclid=CjwKCAjwgajSBhBEEiwASicJU8rhCQR5J3v8heE5vKZAI33EsH1but0NMEYfRWk8Bcy-nOr4prRveRoCf7EQAvD_BwE

Batery: 503035 3.7v https://www.amazon.com/Winfox-Rechargeable-Connector-Bluetooth-Portable/dp/B0GDQLLF12/ref=sr_1_1_sspa?crid=2A5BO56HILG18&keywords=qimoo%2B503035&qid=1783271126&sprefix=qimoo%2B503035%2Caps%2C135&sr=8-1-spons&sp_csd=d2lkZ2V0TmFtZT1zcF9idGY&th=1

Resistors: 2 10k Ohm

Wire: 28 Ga, I reccomend getting at least 4 colors (1 for each of the following 3.3V, GND, SCL, SDA)

20mm watch band: https://www.amazon.com/WOCCI-Adjustable-Nylon-Watch-Release/dp/B09YTPMRJY/ref=sr_1_1?crid=7ZQQV8H5AX59&dib=eyJ2IjoiMSJ9.94SnBy2WNNc4WtQAMlg8hKfPnjmsUvKo0H8wbet5InU7dAmFyfRdwfX3QJoQa7jmQV7MpF9Gq9-BLoQds0j9sB4wRBWyPaTQ_mHYmjQt4egjGgfEjnVN2WczBOdUyuBXh8dnPOdkBaISq2ekIWo8DKzF7QORoaIpDWqZOHLTZof8tbch0wF2cG6oCYlGY5Q2cRiV3lxsHEaWS0uOjeJNHWN-nc9ok0RWL9Zk6vsRTJjTs4-18l1exomIHW6a0TK2DQfYyWTFfixoOP1IV8KhMoNfqvNMrsmX5xfG0eSlQNA._qImbWuItak_NKJGU3LS2PPzDJJfZTXi46O91t-ia0c&dib_tag=se&keywords=wocci%2B20mm%2Badjustable%2Bnylon%2Bwatch%2Bband&qid=1783271051&sprefix=wocci%2B20mm%2Badj%2Caps%2C131&sr=8-1&th=1

Housing: I printed mine on an Ender 3-V2 in PLA with a 0.4mm nozzle
