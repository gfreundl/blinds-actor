# blinds-actor

This is an actor for the Homematic home automation system.
It can adjust the tilt angle on standard venetian blinds.
So you can control your blinds remotely or by any schedule given by home automation system.

### BOM:
- Arduino Pro Mini 3.3 V / 8 Mhz
- CC1101 transceiver module 868 MHz
- Pololu DRV8838 DC motor driver
- chinese gear motor 6 V / 36 RPM
- 10 Ohms resistor (SMD 0603)
- 2 AA cells and holder

It runs on 2 AA batteries, so for reducing quiescent current down to 4uA these changes are recommended:
- the Arduino should be the 3.3V / 8Mhz type
- the LDO and LED on the Arduino are removed
- the 10K resistor on the DRV8838 board is removed
- the DRV8838 is powered only from the EN (enable) line and send to sleep mode after operation

For reducing torque, spin-up and operating current, a 10 Ohms resistor is in series with the DC motor. 
This resistor also works as a shunt to measure the operating current.
The current is proportional to the torque and is used for detecting the end stop position.

Required Arduino Libraries:
- AskSinPP
- EnableInterrupt
- LowPower

!(Drawing.png)

