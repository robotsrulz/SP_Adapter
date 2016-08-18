
Logitech G25/G27 pedals/shifter adapter
=======================================

The idea behind this project is to create open and inexpensive USB HID adapter for Logitech G25 / G27 pedals and shifters based on affordable components. Once you bought (or built) your new shiny and awesome direct drive steering wheel you no longer need your old but still working Logitech wheel. But it parts still can be used as standalone device.

I am aware that some shops already sell similar adapters. Unfortunately these devices are quite expensive and not open source. Also I want to practice a bit with STM32 programming :)

Hardware part is based on STM32F042 microcontroller(TSSOP20), Schematics folder contains Gerber files as well as DipTrace source files. 

![Adapter](Schematics/adapter.png)

Once it's ready I will provide "stable" binary firmware as well as firmware source. Stay tuned...

## Obtaining the board

The board will soon be available to order on [kitnic.it](https://kitnic.it/boards/github.com/robotsrulz/SP_Adapter/). I'm testing data for 1clickBOM Chrome plugin to order all parts from major component suppliers (Digikey, Mouser, RS, Newark, Farnell). I will also sell a small amount of the boards assembled manually by myself.

## License

This project is licensed under the BSD License - see the [LICENSE](LICENSE) file for details
