#include "ros/ros.h"
#include <math.h>

#include "RoboteqDevice.h"
#include "ErrorCodes.h"
#include "Constants.h"
#include <tf/transform_datatypes.h>
#include "tf/transform_broadcaster.h" 
#include "roboteq/roboteq_msg.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <iostream>
using namespace std;

bool imu_in=true;
double th=0.0;
double initial_head=0.0;
double tick_x = 0.0;
double tick_y = 0.0;
double omega_left_dash = 0.0;
double omega_right_dash = 0.0;

void enc_callback(const roboteq::roboteq_msg::ConstPtr& msg)
{
  tick_x = msg->encoder_1;
  tick_y = msg->encoder_2;  
  omega_left_dash = (msg->rpm_1)/8.3; //reduction factor = 8.3
  omega_right_dash = (msg->rpm_2)/8.3;
}
void imu_callback(const geometry_msgs::Pose2D::ConstPtr& msg)
{
  if(imu_in){  
  th = msg->theta;
  //cout<<msg->theta<< "  ";
  //imu_in = false ;   
  }	
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("wheel/odom", 1);
  ros::Subscriber imu_initial_sub = n.subscribe<geometry_msgs::Pose2D>("imu/HeadingTrue",100,imu_callback);
  ros::Subscriber encoder_sub = n.subscribe<roboteq::roboteq_msg>("/roboteq_monitor",1,enc_callback);
  
  //tf::TransformBroadcaster odom_broadcaster;
 
    
  double x = 0.0;
  double y = 0.0;

  double vx = 0.0;
  double vy = -0.0;
  double vth = 0.0;
  double dth = 0.0;
  double dx = 0.0;

  double _PreviousLeftEncoderCounts = 0.0;
  double _PreviousRightEncoderCounts = 0.0;
  double deltaLeft = 0.0;
  double deltaRight = 0.0;
  double omega_left = 0.0;
  double omega_right = 0.0;
  double v_left = 0.0;
  double v_right = 0.0;
  //double ratio_left = 0.0;
  //double ratio_right = 0.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(25);
  while(ros::ok()){
    
    ros::spinOnce();
    //cout<<th<<"   "<<endl;               // check for incoming messages
    current_time = ros::Time::now(); 
    double lengthBetweenTwoWheels = 0.54;

    deltaLeft = -1*(tick_x - _PreviousLeftEncoderCounts)*(2*3.14159265/131431);
    deltaRight = (tick_y - _PreviousRightEncoderCounts)*(2*3.14159265/131431);

    omega_left = omega_left_dash * (3.14159265/30);//rpm to rad/sec
    omega_right = omega_right_dash * (3.14159265/30);

    v_left = omega_left * 0.942/(2*3.14159265); //radius
    v_right = omega_right * 0.942/(2*3.14159265);

    dx =  (deltaLeft + deltaRight)*(0.237/2);
    dth =  (deltaRight - deltaLeft)*(0.237/lengthBetweenTwoWheels);

    vx = ((v_right + v_left) / 2);
    vy = 0;
    vth = ((v_right - v_left)/lengthBetweenTwoWheels);

    double delta_x = (dx * cos(th)) ;
    double delta_y = (dx * sin(th)) ;
   

    x += delta_x;
    y += delta_y;
    

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //Odometry message
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx * cos(th);
    odom.twist.twist.linear.y = vx * sin(th);
    odom.twist.twist.angular.z = vth;
    //publish the message
    odom_pub.publish(odom);
    _PreviousLeftEncoderCounts = tick_x;
    _PreviousRightEncoderCounts = tick_y;

    
    r.sleep();
  }
}
