#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <fstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>


geometry_msgs::Pose2D vel_sa;
using namespace std;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "status_e2o");
  ros::NodeHandle n;

  

  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Pose2D>("chatter", 1);
  ros::Rate loop_rate(100);
  while (ros::ok())
  {
    ifstream myfile("/home/sine/rise_ws-master/src/goal_set/src/status_can_.txt");
    if(myfile.is_open())
    {
          myfile>>vel_sa.x;
          cout<<vel_sa.x<<endl;
          myfile>>vel_sa.theta;
          cout<<vel_sa.theta<<endl;
    }
        myfile.close();
     





    chatter_pub.publish(vel_sa);

    ros::spinOnce();

    loop_rate.sleep();
  

  }  
  return 0;
}
