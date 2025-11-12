# Firwmare for the TCD1304 using FlexPWM

This directory contains an Arduino sketch file and libraries to operate the TCD1304 Sensor using the Teensy 4 in either the two board system, the TCD1304 SPI board plus Teensy 4 Controller Rev 3 or the All-In-One board with FlexPWM support.

The TCD1304 sensor device presents itself to the host computer as a serial port over USB.   The command set and responses are human readable ASCII, including "help" which lists all of the commands.

The adjacent Python directory in this repo, provides a utility TCD1304Controller.py that can serve as a user command interface with real time graphics or as a Python class library.

The contents of the built-in help has been saved to the file [TCD1304.help](TCD1304.help)  which is included in this directory.

