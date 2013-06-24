Self Balancing RoBot (SBR)
================
For more project information check out the [wiki](https://github.com/ricklon/SelfBalancingBot/wiki)


This Self Balancing Robot project is for the Rutgers University  Governor School Course "How to Make a Self Balancing Robot" started by Aatish in 2012. He had the self balancing robot using the Sparkfun box. In addition Jason Doreweller, http://www.jddorweiler.appspot.com/electronics.html#robot,  created a Self Balancing Robot at Fubar Labs in 2012. Concepts from that robot where tested, and modeled developing this new robot.

This project has added:
* servo calibration sketches
* data gathering sketches and test for the accelermeter
* data gathering sketches and test for the gyro
* A servo movement library
* PID, Kalman fileter library

Arduino libraries used in the project:
* Servo
* ITG3200
* ADXL445
* Serial

BOM:
* Teensy 2.0
* 2 Parallax Continuous Servos
* 5v Switching voltage regulator
* Sparkfun 9 DOF freedom IMO

Note: Using the Teensy eliminated the Serial vs Servo timer problem.

Laser cut reference design for a body to mount the project.



Notes for Teensy 2.0
====
[I2C pin out]( http://www.pjrc.com/teensy/pinout2b.png)

[Teensy 2.0 pin mapping](http://www.pjrc.com/teensy/pinout.html)


