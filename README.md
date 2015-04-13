# Yun-I2C-MOD-1016v6-Example
A checkout sketch for the Mod-1016v6 using I2C for Arduino Yun.

## Overview
Nothing groundbreaking, just a working example. 

![MOD-1016 Overview](/images/Yun-I2C-MOD-1016v6-Example.jpg)

## Design Notes
I prefer to use I2C because it simplifies the wiring when connecting multiple devices but the libraries and examples I found were written for SPI. I also prefer the Yun over other Arduinos because the wireless bridge makes it easy to test and troubleshoot.

The AS3935 requires I2C repeated start command and the Arduino wire library can't do that so I'm using SoftI2C instead. For greater flexibility I've also opted to use Pin Change interrupts rather than hardware interrupts. 

When powered by 5V this device seems reluctant to generate any interrupts. Using 3.3V from the Yun works much better.

I included the antenna tuning method from raivisr to confirm the EA tuning on the package. It was in the ballpark but hopped around. Bumping the frequency sample interval to 1s it settles down nicely and almost always agrees with the EA suggested setting.

## Links and Credits
[Embedded Adventures Lightning Sensor Module](http://www.embeddedadventures.com/as3935_lightning_sensor_module_mod-1016.html)

[raivisr/AS3935-Arduino-Library](https://github.com/raivisr/AS3935-Arduino-Library)

[evsc/ThunderAndLightning](https://github.com/evsc/ThunderAndLightning)

[Software I2C library](http://playground.arduino.cc/Main/SoftwareI2CLibrary)

[Simple Pin Change Interrupt on all pins](http://playground.arduino.cc/Main/PinChangeInterrupt)

Thanks also to Adafruit for their I2C library examples.

## Output
```
Unable to connect: retrying (1)... connected!
>Hello World!
>Hello MOD-1016v6!
>Auto Tune
0x0 35 
0x1 28 
0x2 13 
0x3 8 
0x4 4 
0x5 17 
0x6 24 
0x7 34 
0x8 42 
0x9 56 
0xA 61 
0xB 76 
0xC 80 
0xD 88 
0xE 98 
0xF 107 
>Registers
reg 76543210
--- --------
0x0 00100100
0x1 00100010
0x2 11000010
0x3 00000000
0x4 00000000
0x5 00000000
0x6 00000000
0x7 00111111
0x8 00000100
>Listening
23903 INT_NH Noise level too high
30259 INT_D  Disturber detected
```
