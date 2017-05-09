# Alicorn
Alicorn is an ESP8266 software designed to track various sensor data and exchange them with [VariPass](https://varipass.org/) to help track daily changes to temperature, humidity, pressure, air quality, and changes to the Earth's magnetic field. Additionally, the software makes use of a DIY geiger counter to track radiation levels and sounds an alarm if the radiation is too high. Alicorn is able to potentially work for months on a larger external battery by disabling certain features.

## Features
* Measures temperature and humidity using a DHT22 sensor.
* Measures pressure using a BMP180 sensor.
* Measures air quality using an MQ135 sensor.
* Measures Earth's magnetic field using an HMC5883L sensor.
* Measures geiger counts per minute and calculates ratiation dose.
* Sounds an alarm if the radiation exceeds a threshold.
* Syncs data with [VariPass](https://varipass.org/).
* Control and overview using an IR remote and an LCD display.
* On-device settings, persistent through reboots.