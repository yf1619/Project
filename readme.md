
# Control
For control part, the esp32 should communicate with the server and the rover
"esp32\Ardunio"folder is used to store the files with server connection part.

## Requirement
The code given to driver should contain the information about the mode selected : 
Firstly, directly inform the motivation.
Secondly, send the coordinate to the driver.

DRIVER
Sensor could send the information of Angle and Position.

CLK
We should use the same frquency of the sensor( dirver part given );


## Device Information
Then board we use is Espressif ESP32 Dev Module.Usually, the port we use is COM5.
Everyting we observe the serial monitor, the baud rate should be 115200;

## WIFI-Connection 
https://randomnerdtutorials.com/esp32-web-server-arduino-ide/
WIFI has already connected.
header.indexOf("GET /26/on") >= 0   do not understand the grammer of this sentence.


https://randomnerdtutorials.com/esp32-useful-wi-fi-functions-arduino/
Above website is the functions of wifi connections. There are details required to build wifi connection of esp32 boared.


## Ardunio
Everytime open the Ardunio, we could open the sketch to preventing writing the code again.


The ESP32 board can act as Wi-Fi Station, Access Point or both.
