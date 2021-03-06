# Slim Sammy: an autonomous soil collection rover

See [this doc](https://docs.google.com/document/d/1HmI_FHcoloKW9NTQgP__jY4tAkVI42v2Wa3lFLxiRu0/edit) for how to contribute and use the Beaglebone!!!

Once the rocket lands, our rover will be ejected, drive away from the rocket, and collect a soil sample. Our rover is powered by the BeagleBone Blue (an embedded Linux board), and uses two DC motors to drive the wheels, and servos for the scoop and landing correction. 

It has four software systems: ALC (Actuated Landing Correction), Drive, Object Detection and Avoidance (ODAS), and Soil Collection. 

This is currently a work in progress but stay tuned for the finished product in Spring!

## Actuated Landing Correction
The ALC uses accelerometer data to determine which way to turn the rover using a continuous servo. This way, the rover will be oriented correctly after the rocket lands. 

Compile with: 
``` 
gcc ALC.c -lroboticscape -o alc
```
Run with:
```
sudo ./alc
```
