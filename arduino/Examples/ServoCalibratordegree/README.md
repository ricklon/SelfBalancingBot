Servo Calibration
====

Sketch: [Servo Calibration to Degrees](https://github.com/ricklon/SelfBalancingBot/tree/master/arduino/ServoCalibratordegree)

This sketech can be used for two purposes, to Zero the Servos, and to map out the calibration curve (pulse width in Mircoseconds Vs. Observed rotatoinal rate).
 
It was built for use with Parallax Continuous Rotation Servo (#900-00008). The stated max rotation rate is 50 RPM (300 Deg/Sec).  The stated limits on the pulse width are 1300 micro seconds to 1700 micro seconds. 1300 microseconds corresponds to -50 RPM rotation, and 1700 should be +50.  These nomial values are not correct, this code can be used to map out the  pulse with that corresponds to the effective rate by observing some specified values of the rotatio.  
 
This code can take in a value for the pulse witdh  via the serial interface and will run the servo at that rate for 2 seconds. During the "Run" time the led on the teensy will be lit. It will then set the pulse with to the nominal zero value (1500 micro seconds) for 2 seconds. During the "stop" time the led will be off.
 
To zero: Run the sketch and send it an edge value (e.g 1300) via serial so that it's at max rotation speed. Adjust the screws so that when the led is off, the servos are not moving.
 
To find the calibration:  Send a pulse width that is between 1300 and 1500 (e.g 1450).  Mark the servo so that you can observe a full rotation. Adjust this value and observe the servo until you find a value where the servo makes a single complete revolution (I put up a refrence pointing device like a finger or screwdriver to see when the rotation is complete).  At this value the servo is moving at 360 degrees per 2 seconds  (180 deg/sec = 30 RPM).  Repeat for additional angles (180, 90, etc…)  to get the full curve.
