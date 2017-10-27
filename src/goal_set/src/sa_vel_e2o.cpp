#include "ros/ros.h"
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include <sstream>
#include <cmath>
#include <SerialStream.h>
#include <iostream>
#include <unistd.h>
#include <cstdlib>
#include <string>
#include <sensor_msgs/Imu.h>

float theta_curr,theta_initial;
float y= 4;
float res_path = 0.5;
float y_des_i=1;
float x_des,y_des;
float str_ang_cmd;
float theta_err,theta_des,theta_out;
float KP=0.75,KI=0.032,KD=0.1;

float integral = 0;
float diff;
float ittime = 0.3;
float err_prior = 0;
float N;
bool init_head_recieved(true);
int brake_can=0,acc_can=0,str_can=0,gear_can=1;

char final_can_[17],brake_can_[3],acc_can_[3],gear_can_,str_can_[3];
const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;

using namespace std;
using namespace LibSerial;
 ofstream myfile;


geometry_msgs::Point s_v;



void current_heading(const geometry_msgs::Pose2D::ConstPtr& msg){
  theta_curr = msg->theta;
  theta_curr + 180;
  if(init_head_recieved)
  {
    theta_initial = theta_curr;
    init_head_recieved = false;
  }
  theta_curr = theta_curr - theta_initial;
  if(abs(theta_curr)>180) theta_curr=360-abs(theta_curr);

  theta_curr = -theta_curr;
  cout<<" theta_curr "<<theta_curr<<endl;
  cout<<" theta_initial "<<theta_initial<<endl;
}

void car_vel(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == 1)
  {
    s_v.x = 0.5;
    acc_can = 118;
    brake_can = 100;
    gear_can = 2;

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
  else
  {
    s_v.x = 0;
    acc_can = 100;
    brake_can = 190;
    gear_can = 1;   
    for (int i = 0; i<3;i++)
    {
      acc_can_[i] = acc_can/pow(10,(2-i));
      acc_can = acc_can%(int)pow(10,(2-i));
    }
    for (int i = 0; i<3;i++)
    {
      brake_can_[i] = brake_can/pow(10,(2-i));
      brake_can = brake_can%(int)pow(10,(2-i));
    }
    gear_can_ = gear_can; 
  }

}


void sa_vel(const nav_msgs::Path::ConstPtr& msg)
{
  if(!msg->poses.empty())
  {

  for(int i=msg->poses.size()-1;i>=0;i--)
  {

      if(abs(msg->poses[i].pose.position.y - y) <  res_path)
      {
        y_des_i = i;
        cout<<i<<"  "<<msg->poses.size()<<endl;
        break;
      }
  }


  x_des = msg->poses[y_des_i].pose.position.x;
  y_des = msg->poses[y_des_i].pose.position.y;

  theta_des = atan((x_des-20)/(y_des-2));
  theta_err = theta_des - theta_curr*DEG_TO_RAD; 
  integral = integral + theta_err*ittime;
  diff = (theta_err - err_prior)/ittime;

  theta_out = KP*theta_err + KI*integral + KD*diff;
  //str_ang_cmd = (theta_out*40*N)/90;
  err_prior = theta_err;
  theta_out = theta_out*RAD_TO_DEG;
  if (theta_out>40 )
  theta_out = 40;
  else if(theta_out <-40)
     theta_out = -40;


  s_v.y= theta_out;
  str_can = theta_out*0.9 + 140;
  for (int i = 0; i<3;i++)
    {
      str_can_[i] = str_can/pow(10,(2-i));
      str_can = str_can%(int)pow(10,(2-i));
    }
   
  }
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker_e2o");

    
    s_v.x = 0;
    s_v.y = 0;

    
  

  ros::NodeHandle n1,n2,n3;
  ros::Subscriber path_sub = n1.subscribe("/sPath", 1, sa_vel);
   ros::Subscriber sub2 = n3.subscribe("/imu/HeadingTrue_degree", 1, current_heading);
  ros::Subscriber car_heading = n2.subscribe("/newpath_required", 1, car_vel);


  ros::Rate loop_rate(200);
//ros::spin();

  while (ros::ok())
  {
    
    //final_can_ = "/"+gear_can_+"00"+str_can_+brake_can_+acc_can_+"00";
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


  myfile.open("/home/sine/rise_ws-master/src/goal_set/src/final_can_.txt", ios::trunc);
for(int i=0;i<17;i++)
{
  myfile<<final_can_[i];
}
  myfile.close();
/*for(int i=0;i<17;i++)
cout<<final_can_[i];*/
/*
    serial_stream.write(final_can_,16) ;

*/

   ros::spinOnce();

    loop_rate.sleep();

  }
  /*serial_stream.Close() ;*/


  return 0;
}