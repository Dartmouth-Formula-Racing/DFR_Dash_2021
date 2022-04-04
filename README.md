# DFR Dashboard 2021/2022

This repository contains code that handles input and output between the driver and the CVC.

## Hardware
This project is designed to run on the [SparkFun AST-CAN485](https://www.sparkfun.com/products/14483), and uses the [ADM1602N1](https://www.sparkfun.com/products/16397) display. 

## Dependencies

[QwiicSerLCD](https://github.com/fourstix/QwiicSerLCD) - Display communication
[AST Can Library](https://github.com/Atlantis-Specialist-Technologies/AST_CAN_Arduino_Library) - CAN communication

#### Building/Flashing

For production:
Compile and flash the Arduino the same way you would with any other sketch, leaving the `DEBUG` preprocessor directive set to `0`.


For development:
Compile and flash the Arduino the same way you would with any other sketch, setting the `DEBUG` preprocessor directive set to `1` in order to enable serial output.

## License

Should probably add one.
