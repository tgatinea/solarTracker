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

The library Ephem_Soleil.h is used to computes the sun ephemeris.

## Principles

At setup, the GPS module is used to recover the date (day/month/year), time(hour/minutes/seconds), longitude, latitude and altitude.

The GPS time is used to ibitialize the internal time into the arduino (See function getTime()).

Thanks to the library Ephem_Soleil.h, the GPS date, latitude, longitude and altitude combined with the current time provides:
* sunset time
* sundown time
* current azimuth of sun

The compass provides the azimuth of the tracker.

The code computes the sunset, sundown, sun azimuth and tracker azimuth to pilot the 2 relays module to go to Est or West.



The initialTime value is then initialized with the GPS time converted into milliseconds.
The getTime() function return
  
