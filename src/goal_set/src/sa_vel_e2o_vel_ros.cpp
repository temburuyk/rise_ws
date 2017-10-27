#include "ros/ros.h"
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include <std_msgs/Float32MultiArray.h>
#include <sstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
float y;
float y_initial = 1.5;
float res_path = 0.5;
float y_des_i=1;
float x_des,y_des;
float Dincm = 1;

float str_theta_err,str_theta_des,str_theta_out;
float str_KP=0.51,str_KI=0.032,str_KD=-0.3;

float str_integral = 0;
float str_diff;
float str_ittime = 0.3;
float str_err_prior = 0;

float velD_err,velD_des,velD_curr,velD_out=0;
float velD_KP=0.51,velD_KI=0.032,velD_KD=-0.3;

float velD_integral = 0;
float velD_diff;
float velD_ittime = 0.3;
float velD_err_prior = 0;

float sa_scale=0.6,acc_scale,brake_scale,y_scale=1;

int brake_can=0,acc_can=0,str_can=0,gear_can=1,velD_curr_can=0;

char final_can_[17],brake_can_[3],acc_can_[3],gear_can_,str_can_[3];

const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

bool master;

using namespace std;

ofstream myfile_write;


void car_vel_bool(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == 1)
    master = 1;
  else
    master = 0;
}

void e2o_vel(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
  if (master == 1)
  {
    velD_des = msg->data[y_des_i];
    velD_curr = velD_curr_can;
    velD_err = (velD_des - velD_curr)/Dincm; 
    velD_integral = velD_integral + velD_err*velD_ittime;
    velD_diff = (velD_err - velD_err_prior)/velD_ittime;

    velD_out = velD_KP*velD_err + velD_KI*velD_integral + velD_KD*velD_diff;

    velD_err_prior = velD_err;
    velD_out = velD_out*RAD_TO_DEG;
    if (velD_out>0)
    {
      acc_can = velD_out*acc_scale;
      brake_can = 0;
      gear_can = 2;
    }
    else if(velD_out <0)
    {
      acc_can = 0;
      brake_can = velD_out*brake_scale;
      gear_can = 2;
    }
  }
  else
  {
    {
      acc_can = 0;
      brake_can = 90;
      gear_can = 1;
    }
  }
  


  for (int i = 0; i<3;i++)
    {
      acc_can_[i] = acc_can/pow(10,(2-i));
      acc_can = acc_can%(int)(pow(10,(2-i)));
    }
  for (int i = 0; i<3;i++)
    {
      brake_can_[i] = brake_can/(pow(10,(2-i)));
      brake_can = brake_can%(int)pow(10,(2-i));
    }
    
  gear_can_ = gear_can;

}


void e2o_sa(const nav_msgs::Path::ConstPtr& msg)
{
  if(master==1)
  {
  if(!msg->poses.empty())
  {
    y = y_scale*velD_out + y_initial;
    Dincm = 1;
  for(int i=0;i<msg->poses.size();i++)
  {

      if(abs(msg->poses[i].pose.position.y - y) <  res_path)
      {
        y_des_i = i;
        break;
      }
  }
  for(int i=0;i<y_des_i;i++)
    Dincm = Dincm + 100*sqrt(pow((msg->poses[i+1].pose.position.x-msg->poses[i].pose.position.x),2) + pow((msg->poses[i+1].pose.position.y-msg->poses[i].pose.position.y),2));


  x_des = msg->poses[y_des_i].pose.position.x;
  y_des = msg->poses[y_des_i].pose.position.y;

  str_theta_des = atan((x_des-20)/(y_des-2));
  str_theta_err = str_theta_des; 
  str_integral = str_integral + str_theta_err*str_ittime;
  str_diff = (str_theta_err - str_err_prior)/str_ittime;

  str_theta_out = str_KP*str_theta_err + str_KI*str_integral + str_KD*str_diff;

  str_err_prior = str_theta_err;
  str_theta_out = str_theta_out*RAD_TO_DEG;

  if (str_theta_out>40 )
    str_theta_out = 40;
  else if(str_theta_out <-40)
    str_theta_out = -40;


  str_can = str_theta_out*sa_scale + 140;
  }
  }
  else
    str_can = 140;

  for (int i = 0; i<3;i++)
    {
      str_can_[i] = str_can/pow(10,(2-i));
      str_can = str_can%(int)pow(10,(2-i));
    }
   
  
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_e2o_vel_ros");
 

  ros::NodeHandle n1,n2,n3;
  ros::Subscriber path_sub = n1.subscribe("/path", 1, e2o_sa);
  ros::Subscriber car_heading = n2.subscribe("/newpath_required", 1, car_vel_bool);
  ros::Subscriber vel_sub = n3.subscribe("/velocity", 1, e2o_vel);


  ros::Rate loop_rate(200);
//ros::spin();

  while (ros::ok())
  {
   
    final_can_[0]='/';
    final_can_[1]=gear_can_+48;
    final_can_[2]=final_can_[3]='0';
    for(int i = 4;i<7;i++)
      final_can_[i]=str_can_[i-4]+48;
    for(int i = 7;i<10;i++)
      final_can_[i]=brake_can_[i-7]+48;
    for(int i = 10;i<13;i++)
      final_can_[i]=acc_can_[i-10]+48;
    final_can_[13]=final_can_[14]='0';
    final_can_[15]=0x5C;
    final_can_[16]='n';


  myfile_write.open("/home/sine/rise_ws-master/src/goal_set/src/final_can_vel_ros.txt", ios::trunc);
for(int i=0;i<17;i++)
{
  myfile_write<<final_can_[i];
}
  myfile_write.close();

for(int i=0;i<17;i++)
cout<<final_can_[i];

 ifstream myfile_read("/home/sine/rise_ws-master/src/goal_set/src/status_can_.txt"); 
if(myfile_read.is_open())
    {
       
          myfile_read>>velD_curr_can;
            cout<<velD_curr_can<<endl;
        
        myfile_read.close();
    } 


   ros::spinOnce();

    loop_rate.sleep();

  }
 


  return 0;
}