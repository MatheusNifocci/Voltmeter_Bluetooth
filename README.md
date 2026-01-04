#Voltmeter Bluetooth

In this project, I use the Esp32 ADC to measure a voltage signal and send the value via an SPP – Serial Port Profile.

In addition, it is possible to save the measured value in an EEPROM memory. This value is used as a limit, when the measurement exceeds the limit, a red LED is triggered.

You can change the saved value using the BOT button on the ESP32 board.
