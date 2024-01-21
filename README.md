# Line Follower
This is the final project for the Introduction to Robotics course taken in the 3rd year at the Faculty of Mathematics and Computer Science, University of Bucharest.

Team name: **fetitele_powerpuff**
<br>
Team members: [Andreea](https://github.com/AndriciucAndreeaCristina), [Carina](https://github.com/SaicuCarina), [Corina](https://github.com/corinagherasim)

Out of the three tries on the final circuit, our line follower completed the circuit best in *19.65 seconds*.

### Youtube video showing our line follower: <a href= "https://www.youtube.com/watch?v=s3lH5o4y6hk"> Click here </a>

### Components Used:

* 1 x Arduino Uno
* 1 x Mini Breadboard
* 1 x LiPo battery as a power source
* 1 x L293D motor driver
* 1 x QTR-8A reflectance sensor
* 1 x Chassis
* 1 x Ball caster
* 2 x DC motors
* 2 x Wheels
* wires, zip-ties and screws as needed

### Technical Task:
The task involved working with a robotics kit and utilizing provided starter code. Our objective was to construct the machine, integrate six QTR-8A sensors, and fine-tune the PID.

To successfully pass the exam, the line follower had to complete at least one lap within a reasonable time frame. The final grade was determined based on specific time thresholds, with the target being under 20 seconds.

### Process explanation
The program incorporates an automated calibration process executed during startup. This calibration adapts the robot to varying environmental conditions by utilizing data from QTRSensors. As the robot may encounter different color intensities or lighting conditions, this initialization ensures optimal sensor sensitivity and responsiveness.

The core of the line-following functionality lies in the PID (Proportional, Integral, Derivative) algorithm. This algorithm continuously adjusts the motor speeds to keep the robot precisely following the line. The proportional term (kp) helps the robot respond to the immediate error, while the integral term (ki) accumulates past errors to eliminate steady-state discrepancies. The derivative term (kd) anticipates future trends, minimizing abrupt changes. The PID constants (kp, ki, kd) have been fine-tuned through repeated testing on a controlled circuit, ensuring a balance between responsiveness and stability.

### Picture of the setup:
<img src = "https://github.com/corinagherasim/LineFollower/assets/94368761/06eb2dd3-65b9-41da-98e2-f19109bf32e8" width="300" height="500">
