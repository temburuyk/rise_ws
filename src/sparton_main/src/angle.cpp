#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/Imu.h"
#include <ros/subscriber.h>
#include <ros/publisher.h>
#include <ros/node_handle.h>
#include <cmath>

using namespace std;

float roll;

std_msgs::Float64 imu1;

void chatterCallback(const sensor_msgs::Imu& msg)
{

  float q0 = msg.orientation.x;
  float q1 = msg.orientation.y;
  float q2 = msg.orientation.z;
  float q3 = msg.orientation.w;

  // Source wikipedia - Quaternion
  roll = -atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
  roll = roll*180/3.14;
  cout << "Angle = " << roll << endl;

  imu1.data = roll;
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("imu_data", 1, chatterCallback);
  ros::Publisher imu_pub = n.advertise<std_msgs::Float64>("imu_data1", 1);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    imu_pub.publish(imu1);

    ros::spinOnce();

    loop_rate.sleep();
   
  }


  return 0;
}