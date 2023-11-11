# Milestone 3 - Drive-System
Our robot's drive system is achieved using two DC motors attached to an ESP32. To control the robot's speed, direction, and turning, a controller with multiple inputs is built using another ESP32. The two ESP32s are linked via MAC addresses, allowing the controller to send information to the drive motors.  
Two arduino sketches are used.
The controller code is seen in controller.ino and the drive code is seen in drive.ino.
