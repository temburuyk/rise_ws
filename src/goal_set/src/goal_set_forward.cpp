#include "ros/ros.h"
#include "std_msgs/String.h"
#include "stdio.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Bool.h"
#include "gps_common/conversions.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose2D.h>


#include <iostream>
#include <string>
#include <vector>
#include <cmath>

using namespace std;

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

int co_number = 3; //Ensure co_number is not greater than size of des_lat
double des_lat[3] = {19.1358950};//19.132710; 19.1327367167     //Define Required GPS location
double des_lon[3] = {72.9047782};//72.915509;  72.9159374433
double des_xpos=0;
double des_ypos=0;

float stop_xpos = 0;
float stop_ypos = 0;
float stop_xpos_temp = 0;
float stop_ypos_temp = 0;

double curr_lat=0;
double curr_lon=0;
double curr_xpos=0;
double curr_ypos=0;

double dist_x=0;
double dist_y=0;
double dist_r=0;

float curr_head   = 0;
float last_head   = 0;
float des_head = 0;
float map_theta   = 0;


bool map_received(false), goal_set(false), currhead_received(false);

geometry_msgs::PoseStamped goal;

double ach_accuracy= 1.5;
int width = 160;
int height = 120;
float map_res = 0.25;
float angletochange = 0;
float reduce_velforang = 1;
float dist_to_obs = 10;
float dist_to_frt = 3;
float distmax = 3.0;  //reduce = 1
float distmin = 0.4;  //reduce = alphamin
float max_ang_vel = 120;
float plan_vel = 0;
float max_vel_wobs = 1.6;//1.6
float max_vel_obs = 1.3;//1.4
bool new_path_stats = true;
bool one_back = false;
bool st_obj = false; //false-no obj   true-obs


string zone;
string data;

geometry_msgs::Quaternion toQuaternion(double pitch, double roll, double yaw)
{
	geometry_msgs::Quaternion q;
        // Abbreviations for the various angular functions
	double cy = cos(yaw * 0.5);
	double sy = sin(yaw * 0.5);
	double cr = cos(roll * 0.5);
	double sr = sin(roll * 0.5);
	double cp = cos(pitch * 0.5);
	double sp = sin(pitch * 0.5);

	q.w = cy * cr * cp + sy * sr * sp;
	q.x = cy * sr * cp - sy * cr * sp;
	q.y = cy * cr * sp + sy * sr * cp;
	q.z = sy * cr * cp - cy * sr * sp;
	return q;
}

float calc_angle(float dy, float dx){
	float angle(0);
	angle = atan2(dy , dx)*RAD_TO_DEG;
	return angle;
}

void pos_curr(const sensor_msgs::NavSatFix::ConstPtr& info){
	curr_lat = info->latitude;	
	curr_lon = info->longitude;
	//cout<<curr_lat<<" "<<curr_lon<<endl;
		
		gps_common::LLtoUTM(curr_lat,curr_lon,curr_ypos,curr_xpos,zone);
	
}

void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
	curr_head = msg->theta;
	currhead_received = true;
}

void vel_coeff(const nav_msgs::OccupancyGrid::ConstPtr& map_in){
   // reduce = 1;
    width    = map_in->info.width;
    height   = map_in->info.height;
    map_res  = map_in->info.resolution;
    dist_to_obs=5;
}

int main(int argc, char **argv){	
	ros::init(argc, argv, "control_sys");
	float planned_distance = 7;
	float disp_from_centre = 3.0;
	float dispx_from_end = 3.0;
	float dispy_from_end = 23.0;
	ros::NodeHandle n;

	 ros::Subscriber sub   = n.subscribe("/fix", 1, pos_curr);
	 ros::Subscriber sub2 = n.subscribe("/imu/HeadingTrue_degree", 10, current_heading);
	 //ros::Subscriber sub3  = n.subscribe("/scan/blown_local_map", 10, vel_coeff);

	 ros::Publisher pub4  = n.advertise<std_msgs::Bool>("/newpath_required",1);
	 ros::Publisher goal_pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1); 

	 co_number=3;
     des_lat[0]=19.1336270;
     des_lon[0]=72.9131826;
     des_lat[1]=19.1359035;
     des_lon[1]=72.9046810;
     des_lat[3]=19.13575993;
     des_lon[3]=72.90471378;

     gps_common::LLtoUTM(des_lat[0],des_lon[0],des_ypos,des_xpos,zone); 

     std_msgs::Bool path_req;
     std_msgs::Bool st_obj_st;
     std_msgs::Float32 v_des;
     std_msgs::Float32 theta_des;
     std_msgs::Float32 ang_des;

     goal.pose.position.x=0;
  	 goal.pose.position.y=0;
  	 goal.pose.orientation.x=0;
	 goal.pose.orientation.y=0;
	 goal.pose.orientation.z=0;
	 goal.pose.orientation.w=1;
	 goal.header.frame_id="/map";

     path_req.data = true;
     v_des.data=0;
     theta_des.data=0; 
     ang_des.data=max_ang_vel;
     st_obj_st.data=false;

        int co_iterator = 0;
        int count = 0;
ros::Rate loop_rate(10);

    while (ros::ok()){

    	//cout<<"Coorde No:: "<<co_iterator<<endl;

	    dist_x = des_xpos - curr_xpos;
	    dist_y = des_ypos - curr_ypos;
	    dist_r = pow((pow(dist_x,2)+pow(dist_y,2)),0.5); 

 	    if (dist_r < 1.0){  path_req.data = false;  
	                        cout<<"The vehicle is at the destination"<<endl;                                   
	                        }
	    if(dist_r>=1.0){path_req.data = true;}
        pub4.publish(path_req);
        des_head = calc_angle((des_ypos - curr_ypos), (des_xpos - curr_xpos));
        map_theta =  curr_head-des_head;
        goal.pose.orientation = toQuaternion(0 , 0 , (90-map_theta)*DEG_TO_RAD) ; 

        /*if(dist_x<float(width)*map_res/2&&dist_y<float(height)*map_res)
        {
        	goal.pose.position.x = dist_r*cos(map_theta*DEG_TO_RAD)+float(width)*map_res/2;
        	goal.pose.position.y = dist_r*sin(map_theta*DEG_TO_RAD);
        }
        else
        {
        	goal.pose.position.x = distance_to_plan*cos(map_theta*DEG_TO_RAD)+float(width)*map_res/2;
        	goal.pose.position.y = distance_to_plan*sin(map_theta*DEG_TO_RAD); 
        }*/
        cout<<" dist_r "<<dist_r<<endl;
        stop_ypos = int(dist_r*(cos((curr_head-des_head)*DEG_TO_RAD))/map_res);
	 	stop_xpos = (width/2-1) + int((dist_r*sin((curr_head-des_head)*DEG_TO_RAD))/map_res);
	 	if (stop_ypos >= height) stop_ypos = height - 1;
               // }
	 	
	 	else if (stop_ypos < 0) {stop_ypos = 0;}
	 	if (stop_xpos >= width){ stop_xpos = width-1;}
	 	if (stop_xpos < 0) {stop_xpos = 0;}

        /*stop_ypos = (dist_r*cos((curr_head-des_head)*DEG_TO_RAD));
        cout<<" dist_r "<<dist_r<<endl;	 	
	 	stop_xpos = (width*map_res/2) + (dist_r*sin((curr_head-des_head)*DEG_TO_RAD));*/
	 	stop_ypos_temp = height -dispy_from_end/map_res;
	 	cout<<" stop_xpos "<<stop_xpos<<" stop_ypos "<<stop_ypos<<" "<<disp_from_centre<<endl;
	 	cout<<disp_from_centre/map_res<<" "<<disp_from_centre/map_res<<endl;
               // }
	 	if (stop_xpos >= width/2+(int)disp_from_centre/map_res){ stop_xpos_temp = width/2+int(dispx_from_end/map_res);
	 		cout<<" right side"<<endl;
	 		goal.pose.orientation = toQuaternion(0 , 0 , (0)*DEG_TO_RAD) ;
	 	}
	 	if (stop_xpos <= width/2-int(disp_from_centre/map_res)) {stop_xpos_temp = width/2-int(dispx_from_end/map_res);
	 		cout<<"left side"<<endl;
	 		goal.pose.orientation = toQuaternion(0 , 0 , (180)*DEG_TO_RAD) ;

	 	}
	 	if (stop_xpos > width/2-int(disp_from_centre/map_res)&&stop_xpos < width/2+int(disp_from_centre/map_res))
	 	{
	 		cout<<"in middle"<<endl;
	 	 	stop_xpos_temp = width/2;
	 		goal.pose.orientation = toQuaternion(0 , 0 , (90)*DEG_TO_RAD) ;

	 	}




	 	goal.pose.position.x = stop_xpos_temp*map_res;
        goal.pose.position.y = stop_ypos_temp*map_res; 
        cout<<"map_theta "<<map_theta<<"des_head "<<des_head<<"curr_head "<<curr_head<<endl; 
        cout<<"pose x: "<<goal.pose.position.x<<" pose y: "<<goal.pose.position.y <<endl;
        goal_pub.publish(goal);

        
        ros::spinOnce();
		loop_rate.sleep();
     }
	return 0;                    
}
