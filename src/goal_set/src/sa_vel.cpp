#include "ros/ros.h"
#include <iostream>
#include <string>
#include <vector>
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Point.h"
#include <sstream>
#include <cmath>
float y= 1.0;
float res_path=0.5;
float y_des_i=1;
float x_des,y_des;
float str_ang_cmd;
float theta_err,theta_des,theta_out;
float KP=0.8,KI=0.2,KD=0.1;

float integral = 0;
float diff;
float ittime = 0.01;
float err_prior = 0;
float N;
const float pi = 3.14159265;
const float RAD_TO_DEG = 180.0/pi;
const float DEG_TO_RAD = pi/180.0;
using namespace std;

geometry_msgs::Point s_v;
ros::Publisher pub ;


void car_vel(const std_msgs::Bool::ConstPtr& msg)
{
  if (msg->data == 1)
    s_v.x = 0.5;
  else
    s_v.x = 0;

}


void sa_vel(const nav_msgs::Path::ConstPtr& msg)
{
  if(!msg->poses.empty())
  {

  for(int i=0;i<msg->poses.size();i++)
  {

      if(abs(msg->poses[i].pose.position.y - y) <  res_path)
      {
        cout<<" checked "<<endl;
        y_des_i = i;
        break;
      }
  }


  x_des = msg->poses[y_des_i].pose.position.x;
  y_des = msg->poses[y_des_i].pose.position.y;
cout<<"x "<<x_des<<" y "<<y_des<<"y_des_i"<<y_des_i<<endl;
  theta_des = atan((x_des-20)/y_des);
  cout<<"theta_des "<<theta_des<<endl;
  theta_err = (1)*theta_des; 
  integral = integral + theta_err*ittime;
  diff = (theta_err - err_prior)/ittime;

  theta_out = KP*theta_err + KI*integral + KD*diff;
  //str_ang_cmd = (theta_out*40*N)/90;
  err_prior = theta_err;
  theta_out = theta_out*RAD_TO_DEG;
  if (theta_out>17 )
  {
  theta_out = 17;
  }
  if ( theta_out <-17)
  {
  theta_out = -17;
  }
if(theta_out!= 0)
  cout<<theta_out<<endl;

  s_v.y= theta_out;
  }
    pub.publish(s_v);
}

/*void car_heading(const geometry_msgs::Pose2D::ConstPtr& msg)
{


}
*/

int main(int argc, char **argv)
{
  ros::init(argc, argv, "car_out");
    
    s_v.x = 0;
    s_v.y = 0;

  

  ros::NodeHandle n1,n2,n3;
  ros::Subscriber path_sub = n1.subscribe("/path", 1, sa_vel);
  pub = n3.advertise<geometry_msgs::Point>("/sa_vel",1);
  ros::Subscriber car_heading = n2.subscribe("/newpath_required", 1, car_vel);


  ros::Rate loop_rate(10);


  while (ros::ok())
  {
 
    
   



    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}