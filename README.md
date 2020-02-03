# ESP-Humidifer

## Humidifier controller
PlatformIO ESP8266 project  

Control power to a humidifier, a relay turns power to the humidifier on and off to reach the desired setpoint. All interaction is via web interface.

![screenshot](/screenshot.jpg "screenshot")

On initial startup look for HUMIDIFIER on WiFi, select your AP SSID and enter the password.  
IMPORTANT Change the OTA password now.

The device should be connected to WiFi now.  
Browse to http://humidifier.YOURDOMAIN or http://\<IP\>/

## Additional Information
### Settings 
Default relay On/Off minimum time is 15 seconds  

### EEPROM settings
Default PWM WindowSize time is 60 seconds  
WindowSize in mS can be changed by calling http://\<IP\>/window?val=90000  
PID gains may need to be adjusted to get a good response.
Gains can be updated by calling http://\<IP\>/pid?P=2.0&I=0.50&D=0.00  

### Parts Used
 1. ESP8266 D1 Mini https://www.amazon.com/gp/product/B076F53B6S
 1. BME280 Sensor 3.3v https://www.amazon.com/gp/product/B07KYJNFMD
 1. 5V One Channel Relay Module Relay Switch with OPTO Isolation https://www.amazon.com/gp/product/B00LW15A4W
 1. Short micro USB cable https://www.amazon.com/gp/product/B07FDBQZPK
 1. USB Charger (grind down the AC pins until a spade connector fits)
 1. Screw terminal block
 1. Plastic box
 
### Connections
Relay - ESP8266 D1 Mini  
   DC+ -  5V  
   DC- -  GND  
   IN  -  D1  
BME280 - ESP8266 D1 Mini  
    SCA -  D2  
    SCL -  D5  
    VIN -  3V3  
    GND -  GND  


#### Built with
VSCODE, Platformio, Highsoft Highchart

