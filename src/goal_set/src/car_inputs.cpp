#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/Path.h"
#include "visualization_msgs/MarkerArray.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

std_msgs::Bool path_req;
float angle_error = 0;
float max_ang_vel = 100;

const float P_O=0.75, P_WO=0.68, I=0.01, D=0.1;//float P=0.8, I=0.01, D=0.01;
float P=0.6;
float err_new=1, err_old, errd, erri;
float accelerator =0 , steering =0 ,brake = 0;

void update_path_required(std_msgs::Bool msg)
{
	path_req.data = msg.data;
}
void motorcommand(float errd_1, float erri_1){
	float pid = (P*err_new + I*erri_1 + D*errd_1);
	if (pid < (-max_ang_vel)) pid = -max_ang_vel;
	if (pid > max_ang_vel) pid = max_ang_vel;
	cout<<"pid = "<<pid<<endl;
	if(path_req.data)
	{
		accelerator = 10;
		steering = pid;
	}
	else
	{
		accelerator = 0;
		steering = 0;
	}
	
}
void PID(const float error_new){
	// std::cout<<"YO1"<<std::endl;	
//	float yaw = info->linear.theta;

	float dt = 0.05;
	
	err_old = err_new; 
	err_new = error_new;
		
	errd = (err_new-err_old)/dt;
	if (errd < (-20)) errd = -20;
	if (errd > 20) errd = 20;

	erri += (err_new + err_old)*dt/2;	
	if (erri < (-20)) erri = -20;
	if (erri > 20) erri = 20;
	
	motorcommand(errd, erri);
}
void update_angle_error(visualization_msgs::MarkerArray msg)
{
	float orientation1=0 , orientation2 = 0;
	/*if(!msg.markers.empty())
	{
		for(std::vector<visualization_msgs::Marker>::const_iterator i = msg.markers.begin(); i != msg.markers.end(); ++i)
		{
		orientation1 = tf::getYaw(i->pose.orientation);
		orientation2 = tf::getYaw(i->pose.orientation);
		orientation1 = orientation1*RAD_TO_DEG;
		orientation2 = orientation2*RAD_TO_DEG;
		cout<<"orientation1 "<<orientation1<<" orientation2 "<<orientation2<<endl;
		}

	}*/
	if(!msg.markers.empty())
	{
	orientation1 = tf::getYaw(msg.markers[msg.markers.size()-1].pose.orientation);
	orientation2 = tf::getYaw(msg.markers[msg.markers.size()-2].pose.orientation);
	}
	orientation1 = orientation1*RAD_TO_DEG;
	orientation2 = orientation2*RAD_TO_DEG;
	angle_error = orientation2 - orientation1;
	PID(angle_error);
	cout<<"orientation1 "<<orientation1<<" orientation2 "<<orientation2<<endl;

}
void update_path(nav_msgs::Path msg)
{
	if(!msg.poses.empty())
	{
		for(std::vector<geometry_msgs::PoseStamped>::const_iterator i = msg.poses.begin(); i != msg.poses.end(); ++i)
		{
			cout<<i->pose.position.x<<endl;
			cout<<i->pose.position.y<<endl;
			cout<<msg.poses.size()<<endl;
		}
	}
	
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "car_inputs");
	ros::NodeHandle n;
	 ros::Subscriber sub1   = n.subscribe<visualization_msgs::MarkerArray>("/sPathVehicle", 1, update_angle_error);
	 ros::Subscriber sub2 = n.subscribe<std_msgs::Bool>("/newpath_required",1,update_path_required);
	 //ros::Subscriber sub3 = n.subscribe<nav_msgs::Path>("/path",1,update_path);
	 path_req.data = true;
        int count = 0;
		ros::Rate loop_rate(10);
    while (ros::ok()){


        ros::spinOnce();
		loop_rate.sleep();
     }
	return 0;                    
}
