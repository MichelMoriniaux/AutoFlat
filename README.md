# AutoFlat
An Arduino based flat field cover


This flat field cover is compatible with the FlipFlat Alnitak protocol so can be used with any driver that uses it.
It wil emulate a FlipMan or a FlipFlap according to whether a servo is present to move the cap

Currently there is support for 3 board types to build your own system:
- Arduino Nano. This is the original implementation. It has logic to run a PWM port at 31kHz to control a mosfet that in turn will pwm dim a led strip. It can also control a servo to open and close the cover. This wil require a mini usb cable and a second power cable for the servo and led strip. Limitations of this board are that the PWM at that speed is not super stable. EEPROM is used to save the configuration (servo values for the open and closed cap positions) and is wear-levelled
- Seeeduino Xiao. Second iteration. much faster board that allows more flexibility in the pwm speeds with a dedicated function call. Storage uses flash. CAVEAT: does not work with N.I.N.A. the Serial code for the Alnitak driver in N.I.N.A. disables DTR that is needed by the Xiao board. Works fine in Linux Kstars.
- Seeeduino ESP32-C3. Third iteration. Has a dedicate led driver on board. storgae is flash. Works in N.I.N.A.
- PicoPD rp2040. To come: this will be one cable only as the board has a USB-C power distribution that can output 12V. This will be a one cable solution.

##Hardware
- 1KOhm resistor
- IRF520N MOSFET
- 12v led strip neutral white
- 12v 35Kg servo (depending on the weight you want to move)
- Led ceiling light for parts (diffuser)
- 3d printer