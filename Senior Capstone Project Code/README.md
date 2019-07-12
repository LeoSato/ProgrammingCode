# Senior Capstone Project

This code was written for a Senior Capstone Project as a team. The people who were also part of the team are Siyue Huang, Robert Kading, Ashley Wang, and Abdul-Rauf Yakubu.

The goal of the project is to design a device for operating an autonomous vehicle to follow a leader vehicle at a certain distance. A RC car was used as a testbed for the prototype of the device and the device was designed based on the engineering design process.

For the project, the Raspberry Pi 3 operates the code to drive the RC car. The RPi 3 is connected to a RPi 3 camera module and a PWM driver (Adafruit PCA9685). The camera was used to detect a green colored paper (the marker). The PWM driver was used to operate the drive and steering motors with a clean PWM signal. Triangle similarity was used to find the distance from the marker using the focal length of the camera, actual height of the marker, and the pixel height of the bounding box of the marker (with the assumption that the sides of the marker used as the height would always be vertical in the camera image).
