# flight_computer

The `flight_computer` sub-project hosts all material related to the development of the vehicle flight computer.

## Dependencies
- [XCTU](https://hub.digi.com/support/products/xctu/?_gl=1*hjyo6c*_ga*NjAyMjU5NzczLjE3MTU0Mzk4MzM.*_ga_RZXDK3PM3B*MTcxNjg1NTIyMy43LjAuMTcxNjg1NTIyMy42MC4wLjA.) - XBee Pro S1 Setup Software
- [`#include <SoftwareSerial.h>`](https://docs.arduino.cc/learn/built-in-libraries/software-serial/)

## Setup
### Radio Configuration with [XCTU](https://hub.digi.com/support/products/xctu/)

For each XBee radio (two are required, one to interface with ground station and the other to inteface with the flight computer):

1. At the top of the "Radio Configuration" panel, reset each XBee to factory default settings by clicking the `Default` button, followed by the `Write` button if necessary.
2. Change the `ID` ("Pan ID") parameter to whatever number is desired (within the acceptable range of 0x0-0xFFFF) to designate the "Personal Area Network." *This must be the same on both radios.*
3. Change the `CH` ("Channel") parameter to whatever is desired (within the acceptable range of 0xC-0x17). *This must be the same on both radios.*
4. The `DH` and `DL` parameters of each radio must *match* the `SH` and `SL` parameters, respectively, of the other radio. As an example:

Radio | DH | DL | SH | SL |
--- | --- | --- | --- | ---
XBee 1 (Coordinator) | 13A200 | 409842BA | 13A200 | 40928740 |
XBee 2 (End Device) | 13A200 | 40928740 | 13A200 | 409842BA |

- Note: `SH` + `SL` = MAC Address

5. For XBee 1 (the radio that will be connected to ground station), set the `CE` ("Coorinator Enable") parameter to the `Coordinator [1]` selection. For XBee 2 (the radio that will be connected to the flight computer), set the `CE` ("Coorinator Enable") parameter to the `End Device [0]` selection.

6. Write the changes to the radio by clicking the `Write` button at the top of the "Radio Configuration" panel.

The radios should now be configured to communicate wirelessly.

## Troubleshooting
If things are not working (i.e., the radios are not properly communicating), here are some things that have worked in the past:
- Check the voltage of the Arduino power source (7.97 V was not enough, 8.31 V seemed to do the trick)
	- Ultimately just ditched the battery altogether and used a wall-outlet power supply &rarr; no more power issues
- You can use the XCTU serial console to see whether XBee is receiving data or not (good for determining radio issue vs. Arduino issue)
- [Source for difference between XBee `write()` and `print()` methods](https://forum.arduino.cc/t/how-to-send-integers-via-xbee/574266/6)