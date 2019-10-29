# Home Security System

Home security system built using the Texas Instruments Launchpad MSP430FR4133 embedded systems, it must be compiled with the TI development IDE. 

A PCB overlay was built from scratch to serve as the foundation for the security system. Magnetic hall effect sensors monitor entryways and signal intrusion events when the system is armed with a programmable passcode via keypad input. Usage and warning messages display on the embedded system's LCD screen. Interrupt service routines trigger LEDs which represent the status of entryways. An alarm is triggered when the system is armed, an entryway opens and the system isn't disarmed in time.

The system's flow implements a four stage state-machine, implemented in main.c.
