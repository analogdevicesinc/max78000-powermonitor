# Power Monitor Firmware Source for MAX78000 EVKit

Please see the [Benchmarking Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/blob/master/MAX78000_Evaluation_Kit/MAX78000%20Power%20Monitor%20and%20Energy%20Benchmarking%20Guide.pdf) for instructions on how to use the Power Monitor.

The MAX78000 EVKIT is programmed with the Power Monitor Firmware during manufacture.  Building and reflashing the Power Monitor firmware isn't required unless you want to make changes to its function.

#### Build

This firmware can be built on Linux or on Windows using MinGW.  An ARM EABI toolchain is required and should be added to the PATH environment variable.  Alternatively on Windows, the Maxim 1.2.0 SDK contains a toolchain and can be obtained [here](https://www.maximintegrated.com/en/design/software-description.html/swpart=SFW0001500A).

To build the binary image:

`make power_monitor_if`

The resulting binary can be found at build/max32625_if.bin

#### Flashing the Firmware

The Power Monitor MCU contains a USB bootloader that will accept the firmware binary via a virtual USB flash drive.  To activate the bootloader connect the EVKIT to your PC via the J1 USB port (right side of the board) and CN1 (top side of the board).  Power off the board via SW1.  Press and hold SW6 on the EVKIT and then apply power via SW1.  The Power Monitor LED (D4) will turn blue and your computer should enumerate a virtual flash drive called "MAINTENANCE'.

Copy max32625_if.bin onto the drive.  The LED will flash during programming and complete within a few seconds.  Once programming is complete the virtual flash drive will disapper and the Power Monitor firmware will run automatically.

#### Usage

* Open USB COM port associated with J1 on host
* Four single-key commands:
    * 'v' - voltage mode
    * 'i' - current mode
    * 'w' - power mode
    * 't' - triggered mode

All modes emit full-precision version of the data displayed on the LCD.  Units aren't emitted, but are consistant with the LCD.  Labels aren't emitted but the CSV values are in the same order as is displayed on the LCD.  Units and labels are static.


