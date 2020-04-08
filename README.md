# Compliant Stepper Mototr example using a load cell

![](./demo.gif)

## Hardware
* Teensy 3.2 or above
* cheap load cell
* cheap unipolar stepper motor with driver board ( 4 with coil inputs )
* 10nF cap across the loadcell outputs.
* button ( optional, allows for manually calibration of the load-cell at rest )
That's it!

## Pins

* A10-A11 loadcell ( yes they are annoying to get to, but at least they aren't on the underside of the board )
* D4-7 stepper motor contol
* D8 button ( connected buttons other terminal to ground )

This code utlised the teensy's fast ADC differencial comparator across the wheatstone bridge of the loadcell. 


## Why didn't you use the teensy's Programable Gain Amplifier?

I was unable to use the programable gain amplifier of the teensy due to the fact it seems to only work in either 0-1.2v or -1.2-0v. Whereas I wanted to read both positive and negative values across the wheatstone bridge. Although the PGA is adaptive it waits until going above a certain value in the negative threshold before inverting the gain.
