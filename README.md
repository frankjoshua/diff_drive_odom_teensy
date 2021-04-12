# diff_drive_odom_teensy
ROS odom and speed control for a Sabertooth Motor controller with a Teensy 4.0

Sends serial commands to a Sabertooth motor controler over Serial2. Reads encoders using https://github.com/SuperDroidRobots/Encoder-Buffer-Library.
Then sends odometry back to ROS using ROS Serial.

This is just a demo. Not for production use.
