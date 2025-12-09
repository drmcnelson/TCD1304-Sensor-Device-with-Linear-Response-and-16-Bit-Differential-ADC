# Firwmare for the TCD1304 using FlexPWM

This directory contains an Arduino sketch file and libraries to operate the TCD1304 Sensor using the Teensy 4 in either the two board system, the TCD1304 SPI board plus Teensy 4 Controller Rev 3 or the All-In-One board with FlexPWM support.

In this version we add ring buffering.  Regard this as experimental for the time being.   In preliminary testing with an Intel I5 host running Linux, it seems to work well.  We want to test more extensively on a wider range of host platforms.

To setup your system to install the firmware, see the  [Teensyduino website and setup instructions](https://www.pjrc.com/teensy/teensyduino.html)

See the [Python directory in this repo](../../Python) for an interactive user utility with real time graphics.  The Python program uses Python3, Numpy, Matplotlib, PySerial and SciPY.

In the interactive user utility, use the command "help" to see a list of commands.

### In this directory:

The file TCD1304Device2.h  is a header only library for the TCD1304 using the NXP iMXRT1060 FlexPWM.

The file TCD1304Device_Controller_251208.ino  implements the user command interface and instantiates the ring buffer system.

The files parselib2.cpp and parselib2.h provides an api for parsing commands and is used in the controller code to implement the command language for the device.

The ring buffer system is implemented in the controller code, using the frame completion callback interface provide by the TCD1304Device class along with a frame structure also provided in the device library.


