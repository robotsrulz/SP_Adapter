
Logitech G25/G27 pedals/shifter adapter
=======================================

The idea behind this project is to create open and inexpensive USB HID adapter for Logitech G25 / G27 pedals and shifters based on affordable components. Once you bought (or built) your new shiny and awesome direct drive steering wheel you no longer need your old but still working Logitech wheel. But it parts still can be used as standalone device.

I am aware that some shops already sell similar adapters. Unfortunately these devices are quite expensive and not open source. Also I want to practice a bit with STM32 programming :)

"Stable" firmware:

 [bluepill-2.07.hex](binaries/bluepill-2.07.hex)
 
## Wiring G27 shifter

 MOSI  - B5 - orange (5)
 
 MISO  - B4 - gray   (2)
 
 SCK   - B3 - purple (1)
 
 nCS   - A5 - yellow (3)
 
 +3.3V -    - blue   (7)
 
 GND   -    - green  (6)
 
 XAxis - A0 - white  (4)
 
 YAxis - A1 - black  (8)
 
## Pedals

 Throttle - A2
 
 Brake - A3
 
 Clutch - A4

## Calibration software (Windows x86):

 [SP_Profiler.exe](binaries/SP_Profiler_1.2.exe)
 
## Calibration procedure

1. Download and run SP_Profiler.exe.
2. The board should be detected as HID-device in the drop down menu, and red cross should be visible in the calibration area.
3. Press Read button, the blue lines should be visible.
4. The idea of shifter calibration is to match actual shifter X an Y potentiometer positions to gear detection zones, see illustration: ![calibration](https://habrastorage.org/files/a58/dce/3ea/a58dce3ea492499faaae6c0fd2231812.JPG)
You can move blue lines by altering numbers in corresponding edit fields, hit Update button after each edit. Then check if shifter positions are detected correctly. Once you satisfied how shifter detects selected gear press Update and close SP_Profiler, values are stored in adapter flash memory.

## USB VID/PID

The board uses VID 0x1209 (InterBiometrics) / PID 0xF00D to identify itself, see [pid.codes](http://pid.codes/1209/F00D/) database.

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details
