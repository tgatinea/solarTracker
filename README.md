# solarTracker

## Description

The code is dedicated to managed a solar tracker that supports 4 solar pannels (480W peak power each pannel).

The solar tracker frame is composed with 3 car wheels.

One wheel is motorized with an electrical motor of a rolling shutter, a car flywheel and car starter pinion.

The tracker is managed via a hardware composed with:
* arduino uno
* GPS module
* Compass module
* 2 relays module
* LCD screen

The library Ephem_Soleil.h is used to computes the sun ephemeris (Few changes had been done inside this library).

## Principles

At setup, the GPS module is used to recover the date (day/month/year), time(hour/minutes/seconds), longitude, latitude and altitude.

The GPS time is used to initialize an internal time based upon millis() function.

Thanks to the library Ephem_Soleil.h, the GPS date, latitude, longitude and altitude combined with the current time provides:
* sunrise time
* sundown time
* current azimuth of sun

The compass provides the azimuth of the tracker.

The code computes the sunrise, sundown, sun azimuth and tracker azimuth to pilot the 2 relays module to go to Est or West.

Every day at 3 o'clock, the whole hardware is reseted via the watchdog of the arduino to recover the new date and reinitialize the time.

The tracker returns to the Est at the end of the day.

## Tricks

* GPS module is incompatible with arduino delay() and too long Serial.print() as it is needed to pool regulary the serial link of GPS module
* serialSoftware libray cannot be used. Conflict between I2C of compass and serial link of GPS. Workaround: Usage of AltSoftSerial library. https://arduino.stackexchange.com/questions/26240/which-pins-of-an-arduino-uno-can-be-used-as-tx-and-rx-pins-for-connecting-to-gsm#26277
* Factorization of GPS to recover only once a day the date, time, longitude, latitude, altitude. Computation of time locally on arduino via millis().
* Usage of arduino watchdog timer to secure freezing potential bug and also to reset the board every day at 3:00 UTC to resynchronize the time and date with GPS data 
* A forbiden zone is checked to prevent the tracker to turn arround and to mess the electric wires connected to my house. In this zone, a manual intervention is needed to move the tracker and arduino is in permanent reset
* Be careful with 2 relays module to isolate ground from arduino via a separated power supply on JD VCC
* Be carefull with electromagnetic interference: Put arduino, GPS and compass into aluminium box
* Put a capacitor on each 220V wires of the motor to avoid electrical arc on the relays.  
* Compass need to be calibrated. See calibration sketch to find to min/max values for XYZ.
  
