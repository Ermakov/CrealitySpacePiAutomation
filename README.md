# Creality SpacePi automation and control module
This is PCB and Arduino IDE sketch for Creality Space Pi (Plus or not) automation.

Description:
- necessary the module itself and IDC10 female-male cable
- can be inserted into any Creality SpacePi dryer (double, single, built-in Creality CFS or not)
- other functions and cables are optional (backlights, external control and power are not required for operation, but can be connected additionally)
- the cable from the SpacePi display needs to be disconnected from the dryer board and inserted into the "display" socket on the module
- an additional IDC10 cable connecting between the module "mainboard" socket and the dryer board (power is supplied from the dryer board via this cable)
- RJ45 on the module connecting to any I2C master using a regular patch cord and provides data transfer in both directions, and also serves to organize lighting (24V led strip) inside the CFS or dryer
- two pins terminal block on the module used for conning  the LED strip; pad J4 on the back of the module (via diode) used for supply 24 volts to the strip directly from the CFS (without connecting a patch cord with external power)

Module/sketch features:
- interception of all signals between the display/buttons and the dryer board, the module sees the readings on the display and can control the dryer buttons
- automatic switching on SpacePi after power is supplied
- automatic switching on SpacePi drying mode to the values on the display when the module sees humidity above the set point (by default 30%, switching on for 4 hours, then a break of 4 hours, and so on)
- solving the problem with the E4 error (when the dryer does not reach the planned temperature within 10 minutes) by restarting the drying mode every 9 minutes
- transferring data on temperature/humidity in the dryer via I2C as HTU21D sensor protocol
- any information from the dryer display (including an error), as well as pressing any buttons using special I2C commands (see the sketch)

