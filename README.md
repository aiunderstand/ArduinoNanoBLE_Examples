# ArduinoNanoBLE_Examples

This unpolished project was created to demonstrate several capabilities offered by the Arduino Nano BLE (and Sense) boards for USN's Smart Systems course. I use the Nordic Nrf Connect android app to send Int8 message to the board which has a servo wired to it and interprets the message as an angle.

The recommended IDE is PlatformIO and recommended hardware debugger is the Segger J-link EDU. Recommended signal debugger is Digilent Analog Discovery 2.

List of demonstrated functionality:

- button using interupt
- speaker playing the knightrider theme song
- 0-180 degree servo motor
- Light Dependent Resistor (LDR) using analog reads
- BLE server and client
- Segger j-link hardware debugging with conditional breakpoints
- IMU reading 
- Serial monitor on PlatformIO
- Fully wireless microcontroller using power circuit board and battery
- Unity Android app for receiving lidar and IMU signals over BLE for localisation of robot in 3D space.

The code is not polished in any way and is just a rough listing. It should be refactored by the reader.

