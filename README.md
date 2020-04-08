# Compliant Stepper Motor example using a load cell

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

## Installation and usage

Download or clone this repo, and open in platformio. This can be compiled with the Arduino IDE, however you'll need to install the 3rd party libraries mentioned in the platformio.ini manually.

## Code

This code utlised the teensy's fast ADC differencial comparator across the wheatstone bridge of the loadcell. 

The Teensy is able to read the load cell extremely quickly, depending on settings I was getting between 1khz-100khz. Admitidly only with 16bit precision, but this is more than enough for detecting force being applied to it. I've opted for very high averaging as the values are extremely noisy.

### Filtering

I've tried using both a first order ( low pass ) filter, which worked well. I switched to a Kalman filter, and it has marginally better results, increasing responce times while filtering slightly better.  However the benefits are minor.


## Why didn't you use the teensy's Programable Gain Amplifier?

I was unable to use the programable gain amplifier of the teensy due to the fact it seems to only work in either 0-1.2v or -1.2-0v. Whereas I wanted to read both positive and negative values across the wheatstone bridge. Although the PGA is adaptive it waits until going above a certain value in the negative threshold before inverting the gain.
