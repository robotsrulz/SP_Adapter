
Logitech G25/G27 pedals/shifter adapter
=======================================

The idea behind this project is to create open and inexpensive USB HID adapter for Logitech G25 / G27 pedals and shifters based on affordable components. Once you bought (or built) your new shiny and awesome direct drive steering wheel you no longer need your old but still working Logitech wheel. But it parts still can be used as standalone device.

I am aware that some shops already sell similar adapters. Unfortunately these devices are quite expensive and not open source. Also I want to practice a bit with STM32 programming :)

Hardware part is based on STM32F042 microcontroller(TSSOP20), Schematics folder contains Gerber files as well as DipTrace source files.


![Adapter](Schematics/adapter.jpg)

"Stable" firmware:

 [hse-xy-mute-1.5.dfu](binaries/hse-xy-mute-1.5.dfu)
 
 [hse-xy-mute-1.5.hex](binaries/hse-xy-mute-1.5.dfu)
 
## Firmware update

1. Disconnect the device and open the case.
2. Place jumper BOOT0 to right position (assuming you're holding the board with USB connector down and DB9 connectors left and right. Normal position of BOOT0 jumper is left side).
3. Install STM DfuSe, download it here: https://goo.gl/JMUFBS (free registration required). Alternatively install it from my Google Drive: https://goo.gl/dp4XGH
4. Connect the board to computer, it should be detected as "STM device in DFU mode"
5. Download firmware .dfu file.
6. Run DfuSe, mark Verify checkbox, press Choose button, select firmware image, then press Upgrade, agree to the warning message.
7. When Progress bar turns blue firmware update is complete, disconnect the board, move jumper BOOT0 back to the left position, close the case, connect it to computer, the board should be detected as "Shifter/Pedals Adapter". Firmware update complete!

Calibration software (Windows x86):

 [SP_Profiler_1.2.exe](binaries/SP_Profiler_1.2.exe)
 
## Calibration procedure

1. Download and run SP_Profiler.exe.
2. The board should be detected as HID-device in the drop down menu, and red cross should be visible in the calibration area.
3. Press Read button, the blue lines should be visible.
4. The idea of shifter calibration is to match actual shifter X an Y potentiometer positions to gear detection zones, see illustration: ![calibration](https://habrastorage.org/files/a58/dce/3ea/a58dce3ea492499faaae6c0fd2231812.JPG)
You can move blue lines by altering numbers in corresponding edit fields, hit Update button after each edit. Then check if shifter positions are detected correctly. Once you satisfied how shifter detects selected gear press Update and close SP_Profiler, values are stored in adapter flash memory.

## Assigning controls in the game (ver 1.5)

If you cannot assign controls correctly in the game because X and Y axis move with gear selection then press the following buttons on shifter simultaneously: red left + red right + d-pad up. The X and Y axis then lock and you will be able to assign gears in the game. This functionality will be changed soon...

## Obtaining the board

PCB is available to order on [kitnic.it](https://kitnic.it/boards/github.com/robotsrulz/SP_Adapter/). I'm testing data for 1clickBOM Chrome plugin to order all parts from major component suppliers (Digikey, Mouser, RS, Newark, Farnell). I'm also selling a small amount of the boards assembled manually by myself, contact me if you want one.

## USB VID/PID

The board uses VID 0x1209 (InterBiometrics) / PID 0xF00D to identify itself, see [pid.codes](http://pid.codes/1209/F00D/) database.

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details
