#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/Imu.h"
#include <tf/tf.h>

#include <stdio.h>      /* printf, scanf, puts, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <math.h>

#include <sstream>

#define M_PI  3.14159265358979323846


float sonarF_val, sonarFL_val, sonarFR_val, sonarL_val, sonarR_val;
double roll, pitch, yaw;
double d = 0.05; // distance between consecutive sonars

void sonarFrontCallback(const sensor_msgs::Range& msg){

   sonarF_val = msg.range;
   //ROS_INFO_STREAM("Front Sonar's indication: "<<sonarF_val);
}

void sonarFrontLeftCallback(const sensor_msgs::Range& msg){

   sonarFL_val = msg.range;
   //ROS_INFO_STREAM("Front-Left Sonar's indication: "<<sonarFL_val);
}

void sonarFrontRightCallback(const sensor_msgs::Range& msg){

   sonarFR_val = msg.range;
   //ROS_INFO_STREAM("Front-Right Sonar's indication: "<<sonarFR_val);
}

void sonarLeftCallback(const sensor_msgs::Range& msg){

   sonarL_val = msg.range;
   //ROS_INFO_STREAM("Left Sonar's indication: "<<sonarL_val);
}

void sonarRightCallback(const sensor_msgs::Range& msg){

   sonarR_val = msg.range;
   //ROS_INFO_STREAM("Right Sonar's indication: "<<sonarR_val);
}

void imuCallback(const sensor_msgs::Imu& msg){

  tf::Quaternion q(
  msg.orientation.x,
  msg.orientation.y,
  msg.orientation.z,
  msg.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(roll, pitch, yaw);
}

double deg2rad(const int deg){

  double rad = deg * (M_PI/180);

  return rad;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "random_walker");
  ros::NodeHandle n;

  ros::Subscriber sonarFront_sub = n.subscribe("/sensor/sonar_F", 1, &sonarFrontCallback);
  ros::Subscriber sonarFrontLeft_sub = n.subscribe("/sensor/sonar_FL", 1, &sonarFrontLeftCallback);
  ros::Subscriber sonarFrontRight_sub = n.subscribe("/sensor/sonar_FR", 1, &sonarFrontRightCallback);
  ros::Subscriber sonarLeft_sub = n.subscribe("/sensor/sonar_L", 1, &sonarLeftCallback);
  ros::Subscriber sonarRight_sub = n.subscribe("/sensor/sonar_R", 1, &sonarRightCallback);
  ros::Subscriber imu_sub = n.subscribe("/imu", 1, &imuCallback);
  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("/mymobibot/cmd_vel", 1);

  int rosFreqHz;
  n.getParam("/random_walker/rate",rosFreqHz);
  ros::Rate loop_rate(rosFreqHz);

  float linX_vel, angZ_vel;
  n.getParam("/random_walker/linX",linX_vel);
  n.getParam("/random_walker/angZ",angZ_vel);
  double walking_minutes;
  n.getParam("/random_walker/walking_minutes",walking_minutes);

  double fsfw; // front sonar's threshold under which the linear motion is stopped (35cm from front sonar)
  n.getParam("/random_walker/fsfw",fsfw);
  double flsfw; // front-left sonar's threshold under which the linear motion is stopped (25cm from front sonar)
  n.getParam("/random_walker/flsfw",flsfw);
  double frsfw; // front-right sonar's threshold under which the linear motion is stopped (25cm from front sonar)
  n.getParam("/random_walker/frsfw",frsfw);

  /** PHASE's ID EXPLANATION
  1. move forward
  2. rotate cw
  3. rotate ccw
  */
  int phase = 1;
  float linX = 0.0;
  float angZ = 0.0;
  double counter = 0;
  double timeTarget = 0;
  double counterTotal = 0;
  double timeTotalSim = walking_minutes*60;

  sonarF_val = 1000;
  sonarFL_val = 1000;
  sonarFR_val = 1000;
  sonarL_val = 1000;
  sonarR_val = 1000;

  /* initialize random seed: */
  srand (time(NULL));

  ROS_INFO_STREAM("Start of random walking...");

  while ( (ros::ok()) && (counterTotal <= timeTotalSim) )
  {

    counterTotal += 1 /(double)rosFreqHz;

    geometry_msgs::Twist velocity;

    if (phase==1) {

      if ( (sonarF_val < fsfw) || (sonarFL_val < flsfw) || (sonarFR_val < frsfw) ) {

        linX = 0.0;
        angZ = 0.0;

        double rand_val = M_PI/2 + deg2rad(rand()%60 - 30);
        counter = 0;
        timeTarget = rand_val / angZ_vel;

        if (sonarFL_val < sonarFR_val)
          phase = 2;
        else if (sonarFL_val >= sonarFR_val)
          phase = 3;
      }
      else {

        linX = linX_vel;
        angZ = 0.0;
      }
    }

    else if (phase==2) {

      counter += 1 /(double)rosFreqHz;

      if (counter > timeTarget){

        linX = 0.0;
        angZ = 0.0;
        phase = 1;
      }
      else {


        linX = 0.0;
        angZ = angZ_vel;
      }

    }

    else if (phase==3) {

      counter += 1 /(double)rosFreqHz;

      if (counter > timeTarget){

        linX = 0.0;
        angZ = 0.0;
        phase = 1;
      }
      else {

        linX = 0.0;
        angZ = -angZ_vel;
      }

    }

    //ROS_INFO_STREAM("Phase: "<<phase);
    //ROS_INFO_STREAM("Linear Velocity: "<<linX);
    //ROS_INFO_STREAM("Angular Velocity: "<<angZ);

    velocity.linear.x = linX;
    velocity.angular.z = angZ;
    velocity_pub.publish(velocity);

    loop_rate.sleep();
    ros::spinOnce();
  }

  geometry_msgs::Twist velocity;
  velocity.linear.x = 0.0;
  velocity.angular.z = 0.0;
  velocity_pub.publish(velocity);
  loop_rate.sleep();

  ROS_INFO_STREAM("End of random walking...");

  return 0;
}
