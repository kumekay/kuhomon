# Kumekay Home Monitoring Device

System for measurement CO2/Humidity/Temperature/Pressure with OLED display and data upload through WiFi

## Components

- CO2 Sensor MH-Z19
- ESP8266 (NodeMCU ESP12+ based)
- SSD1306 0.96" 128x64 i2c OLED. Library: <https://github.com/olikraus/u8g2>
- Humidity/Temperature SI7021 i2c. Library: <https://github.com/LowPowerLab/SI7021>
- Pressure/Temperature BMP085 i2c. Library: <https://github.com/adafruit/Adafruit-BMP085-Library>

## Case

Model for printing kuhomon_case.stl

Known issues:
- To fit system to this case you have to use short wires
- Temperature measurements seems to be higher due to heat from ESP8266

## TODO
- Add offline support (show data on screen without WiFi)
