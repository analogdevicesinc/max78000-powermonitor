# Power Monitor Firmware Source for MAX78000 EVKit

Please see the [Benchmarking Guide](https://github.com/MaximIntegratedAI/MaximAI_Documentation/blob/master/MAX78000_Evaluation_Kit/MAX78000%20Power%20Monitor%20and%20Energy%20Benchmarking%20Guide.pdf) for instructions on how to use the Power Monitor.



#### Build

Pull these files into the Maxim 1.2.0 SDK that supports the MAX32625.

\<SDK ROOT\>/Firmware/MAX32625/Applications/EvKitExamples

`make power_monitor_if`



#### Usage

* Open USB COM port associated with J1 on host
* Four single-key commands:
    * 'v' - voltage mode
    * 'i' - current mode
    * 'w' - power mode
    * 't' - triggered mode

All modes emit full-precision version of the data displayed on the LCD.  Units aren't emitted, but are consistant with the LCD.  Labels aren't emitted but the CSV values are in the same order as is displayed on the LCD.  Units and labels are static.
