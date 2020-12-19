#include <Arduino.h>
#include <SPI.h>
#include "Kinematics.h"
#include "Encoder.h"
#include "PID.h"
#include "Motor.h"
#include "Odometry.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "geometry_msgs/Twist.h"

#define EncoderCSLeft 6
#define EncoderCSRight 5
#define K_P 5.6  // P constant
#define K_I 0.0  // I constant
#define K_D 40.0 // D constant

//define your robot' specs here
#define TICKS_PER_REVOLUTION 64000 // Number of encoder ticks for full rotation
#define MAX_RPM 330                // motor's maximum RPM
#define COUNTS_PER_REV 1550        // wheel encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.15        // wheel's diameter in meters
#define PWM_BITS 8                 // PWM Resolution of the microcontroller
#define LR_WHEELS_DISTANCE 0.235   // distance between left and right wheels
#define FR_WHEELS_DISTANCE 0.30    // distance between front and rear wheels. Ignore this if you're on 2WD/ACKERMANN

Kinematics kinematics(Kinematics::DIFFERENTIAL_DRIVE, MAX_RPM, WHEEL_DIAMETER, FR_WHEELS_DISTANCE, LR_WHEELS_DISTANCE);
PID pidLeft(-100, 100, K_P, K_I, K_D);
PID pidRight(-100, 100, K_P, K_I, K_D);
Encoder encoder(EncoderCSLeft, EncoderCSRight, WHEEL_DIAMETER / 2, TICKS_PER_REVOLUTION);
Motor leftMotor(2);
Motor rightMotor(1);
Odometry odometry;

void commandCallback(const geometry_msgs::Twist &cmd_msg);
ros::Subscriber<geometry_msgs::Twist> cmd_sub("cmd_vel", commandCallback);
ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[] = "/base_footprint";
char odom[] = "/odom";
long lastCommandTime = 0;
Kinematics::rpm goalRPM;

geometry_msgs::Twist velTwist;
ros::Publisher velPub("raw_vel", &velTwist);

void commandCallback(const geometry_msgs::Twist &cmd_msg)
{
  //callback function every time linear and angular speed is received from 'cmd_vel' topic
  //this callback function receives cmd_msg object where linear and angular speed are stored
  lastCommandTime = millis();
  goalRPM = kinematics.getRPM(cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z);
}

void setup()
{
  Serial.begin(115200);

  //Setup ROS
  nh.initNode();
  nh.subscribe(cmd_sub);
  nh.advertise(velPub);
  broadcaster.init(nh);
  while (!nh.connected())
  {
    nh.spinOnce();
  }
  // goalRPM = kinematics.getRPM(0.2, 0, 0);
}

void loop()
{
  delay(10);

  if (millis() - lastCommandTime > 400)
  {
    goalRPM = kinematics.getRPM(0, 0, 0);
  }

  Encoder::RPM rpm = encoder.getRPM();
  leftMotor.adjust(pidLeft.compute(goalRPM.motor1, rpm.left));
  rightMotor.adjust(pidRight.compute(goalRPM.motor2, rpm.right));
  Kinematics::velocities vel = kinematics.getVelocities(rpm.left, rpm.right, 0, 0);
  Odometry::Position position = odometry.calculatePosition(vel.linear_x, vel.angular_z);

  // tf odom->base_link
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = position.x;
  t.transform.translation.y = position.y;
  t.transform.rotation = tf::createQuaternionFromYaw(position.theta);
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();

  velTwist.linear.x = vel.linear_x;
  velTwist.angular.z = vel.angular_z;
  velPub.publish(&velTwist);
}