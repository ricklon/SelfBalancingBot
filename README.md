Self Balancing RoBot (SBR)
================
For more project information check out the [wiki](https://github.com/ricklon/SelfBalancingBot/wiki)

- [ ] Need to upgrade to faster motors
- [ ] Keep the servo style control
- [ ] May have to Create a a custom servo library
- [ ] Test with hobby gearnotor
- [ ] PID needs to coordinate 2 motors
- [ ] Would like to add support for mpu6500 and mpu9250
- [ ] Add multiple core support [Teensy 3.x, Fubarino Mini 2.0, esp32, esp8266]
- [ ] Motor controller l293d
- [ ] Evaluate power supply

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


