# Power Monitor Firmware Source for MAX7800x EVKit

Please see the [Benchmarking Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/blob/master/Guides/MAX7800x%20Power%20Monitor%20and%20Energy%20Benchmarking%20Guide.md) for instructions on how to use the Power Monitor.

The EVKit is programmed with the Power Monitor Firmware during manufacture.  Building and reflashing the Power Monitor firmware is not required unless you want to make changes to its function.

#### Build

This firmware can be built using the Analog Devices MSDK toolchain:

  * [Windows](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0010820A)
  * [Ubuntu Linux](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018720A)
  * [macOS](https://www.analog.com/en/design-center/evaluation-hardware-and-software/software/software-download?swpart=SFW0018610A)

The target EVKit is specified by the Makefile variable “EVKIT”. Valid EVKIT values are “MAX78000” and “MAX78002”.

To build the MAX78002EVKit binary image:

`make power_monitor_if EVKIT=MAX78002`

The resulting binary can be found at *build/max32625_pmon_max78002evkit_if.bin*

#### Flashing the Firmware

The Power Monitor MCU contains a USB bootloader that will accept the firmware binary via a virtual USB flash drive.

To load a PMON firmware image:

* Connect the dedicated PMON USB port (J1 on MAX78000EVIT, or CN1 on MAX78002EVKIT) to your computer.
* Put the PMON MCU into boot loader mode by pressing the PWR MODE SEL LEFT pushbutton (SW6 on MAX78000EVKIT and SW2 on MAX78002EVKIT) while power cycling the board.
* Wait for 625 STATUS LED (D4 on MAX78000EVKIT, or D8 on MAX78002EVKIT) to illuminate.
* A virtual drive will appear on your host computer.
* Drag and drop the PMON firmware image onto the drive.
* Once the drive disappears, you may need to power cycle the board.
* The PMON firmware should display power information a second or two after reset.
