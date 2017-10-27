//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
#include<string>
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
using namespace std;
using namespace cv;

#include <polysync_ros_bridge/ps_msg_header.h>
#include <polysync_ros_bridge/ps_sensor_descriptor.h>
#include <polysync_ros_bridge/ps_radar_targets_msg.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

ros::Publisher gridmap_pub;

void radarCallback(const polysync_ros_bridge::ps_radar_targets_msg::ConstPtr& msg)
{
nav_msgs::OccupancyGrid m;
geometry_msgs::Quaternion z;

//ROS_INFO("I heard1:");
z.x=0;
z.y=0;
z.z=0;
z.w=0;
int width=1000;
int height=1000;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
float resolution=0.09;
m.info.resolution=resolution;
m.info.origin.position.x=0 ;
m.info.origin.position.y=0 ;
m.info.origin.orientation=z;
float a[63];
float b[63];
int nooftargets=0;
m.info.width=width;
m.info.height=height;
int targetno[63];
//ROS_INFO("I heard2:");
for(int i=0;i<64;i++)
{
  if(msg->targets[i].velocity[0]>0&&msg->targets[i].velocity[1]>0)
  {
    float r=msg->targets[i].velocity[0];
    float t=msg->targets[i].velocity[1];
    float y=msg->targets[i].track_status.value;
    float q=msg->targets[i].position[0];
    float w=msg->targets[i].position[1];
    float e=msg->targets[i].amplitude;
    cout<<r<<" "<<t<<" "<<y<<" "<<q<<" "<<w<<" "<<e<<endl;
    a[nooftargets]=msg->targets[i].position[0];
    b[nooftargets]=msg->targets[i].position[1];
    targetno[nooftargets]=i;
    nooftargets=nooftargets+1;
  }
}
//ROS_INFO("I heard positions:%f,%f",a[0],b[0]);
for(int u=0;u<nooftargets;u++)
 {
  a[u]=a[u]/resolution;
  b[u]=b[u]/resolution;
 }
//ROS_INFO("I heard6:%f,%f",a[0],a[0]);
int c[nooftargets];
int d[nooftargets];
for(int u=0;u<nooftargets;u++)
 {
  c[u]=round(a[u]);
  d[u]=round(b[u]);
 } 
//ROS_INFO("I heard3:%d,%d",c[0],d[0]);
//a and b is in meters
//a=a/resolution
for(int j=0;j<width;j++)
 {
  for(int k=0;k<height;k++)
   {
    m.data.push_back(0);
   }
 }
Mat local_map = Mat::zeros(width,height,CV_8UC1);
//ROS_INFO("I heard5:");
for(int u=0;u<nooftargets;u++)
 {
   if(a[u]<width/2&&b[u]<height/2)
    {
     m.data[width*(height/2)+width/2-c[u]*height-d[u]]=100;
     //m.data[b*500+a]=100;
    }
 }

for(int s=0;s<width;s++)
 {
     for(int q=0;q<height;q++)
      { 
       local_map.at<uchar>(s,q) = m.data[s*width+q]*2.55;
      }
 }
for(int u=0;u<nooftargets;u++)
 {
  if(a[u]<width/2&&b[u]<height/2)
    {
      int y=msg->targets[targetno[u]].track_status.value;
      stringstream ss;
      ss << y;
      string str = ss.str();
      putText(local_map,str,cvPoint(width/2-d[u],height/2-c[u]), FONT_HERSHEY_DUPLEX, 1, cvScalar(200), 1, 8, false );
    }
 }
//Mat large_image;
imshow("large_image",local_map);
//resize(local_map,large_image,cvSize(1000,1000));
//  imshow("large_image",large_image);
waitKey(1);
gridmap_pub.publish(m);    
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "grid");


  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("polysync/ps_radar_targets_msg",1, radarCallback );
//ros::Subscriber sub2 = n.subscribe("chatter",1000, chatterCallback );
gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("gridmap", 1);
 int count = 0;
ros::Rate loop_rate(50);

ROS_INFO("I heard4");
  while (ros::ok())
  {
  ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
 

  return 0;
}

