## SAM3X8E Driver

A grblHAL driver for the Atmel SAM3X8E processor on a [Arduino Due board](https://store.arduino.cc/arduino-due).

This driver can be built with the [Web Builder](http://svn.io-engineering.com:8080/?driver=SAM3X8E).

For local builds see the Wiki-page for [compiling grblHAL](https://github.com/grblHAL/core/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](grblHAL_Due/src/my_machine.h).

Pin mappings for a selection of shields/boards can be found [here](https://github.com/grblHAL/SAM3X8E/tree/master/Arduino%20Due%20shield%20pin%20mappings).

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework.

Note: There is a hardcoded 10ms delay in the Arduino SAM core's CDC transmit codepath that causes very poor performance when using the native USB interface, see grblHAL forum discussion [here](https://github.com/grblHAL/core/discussions/855#discussioncomment-15141314).  Commenting out the delay in your Arduino SAM core as shown in this [issue](https://github.com/arduino/ArduinoCore-sam/issues/106) / [pr](https://github.com/arduino/ArduinoCore-sam/pull/107) fixes the issue.

---

CNC breakout boards:

[Arduino Due shield](https://github.com/itadinanta/cnc_mill_prototype/tree/master/arduino_duo_shield) by Nico Orr&ugrave;.

---
2023-09-20
