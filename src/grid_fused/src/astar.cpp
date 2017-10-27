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
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include<math.h>
using namespace std;
using namespace cv;

#include <polysync_ros_bridge/ps_msg_header.h>
#include <polysync_ros_bridge/ps_sensor_descriptor.h>
#include <polysync_ros_bridge/ps_radar_targets_msg.h>
ros::Publisher gridmap_pub;

void search(Mat& gvalue, Mat& tick_grid,float i,float j,int width,int height,int g,int& reached_end,int end_x,int end_y,Mat cost_function)
{
	cout<<"hello "<<i<<" "<<j<<" "<<endl;
tick_grid.at<uchar>(i,j)=1;
gvalue.at<float>(i,j)=g+1;
g=g+1;
float a=cost_function.at<float>(i+1,j);
float b=cost_function.at<float>(i-1,j);
float c=cost_function.at<float>(i,j+1);
float d=cost_function.at<float>(i,j-1);
      cout<<a<<" "<<b<<" "<<c<<" "<<d<<endl;
if(end_x==i&&end_y==j)
{
reached_end=1;
}
if(reached_end==1)
{
return;
}
cout<<reached_end<<endl;
if(i+1>=0&&i+1<width)
{
if(!reached_end&&tick_grid.at<uchar>(i+1,j)==0&&cost_function.at<float>(i+1,j)<=cost_function.at<float>(i-1,j)&&cost_function.at<float>(i+1,j)<=cost_function.at<float>(i,j+1)&&cost_function.at<float>(i+1,j)<=cost_function.at<float>(i,j-1))
{cout<<"right"<<endl;
search(gvalue,tick_grid,i+1,j,width,height,g,reached_end,end_x,end_y,cost_function);
}
}
if(i-1>=0&&i-1<width)
{
if(!reached_end&&tick_grid.at<uchar>(i-1,j)==0&&cost_function.at<float>(i-1,j)<cost_function.at<float>(i+1,j)&&cost_function.at<float>(i-1,j)<=cost_function.at<float>(i,j+1)&&cost_function.at<float>(i-1,j)<=cost_function.at<float>(i,j-1))
{cout<<"left"<<endl;
search(gvalue,tick_grid,i-1,j,width,height,g,reached_end,end_x,end_y,cost_function);
}
}
if(j+1>=0&&j+1<height)
{
	if(!reached_end&&tick_grid.at<uchar>(i,j+1)==0&&cost_function.at<float>(i,j+1)<cost_function.at<float>(i-1,j)&&cost_function.at<float>(i,j+1)<cost_function.at<float>(i+1,j)&&cost_function.at<float>(i,j+1)<=cost_function.at<float>(i,j-1))
{cout<<"down"<<endl;
search(gvalue,tick_grid,i,j+1,width,height,g,reached_end,end_x,end_y,cost_function);
}
}
if(j-1>=0&&j-1<height)
{
	if(!reached_end&&tick_grid.at<uchar>(i,j-1)==0&&cost_function.at<float>(i,j-1)<cost_function.at<float>(i-1,j)&&cost_function.at<float>(i,j-1)<cost_function.at<float>(i,j+1)&&cost_function.at<float>(i,j-1)<cost_function.at<float>(i+1,j))
{cout<<"up"<<endl;
search(gvalue,tick_grid,i,j-1,width,height,g,reached_end,end_x,end_y,cost_function);
}
}
return;
}
void path_planning(Mat local_map)
{
int width=local_map.cols;
int height=local_map.rows;
int reached_end=0;
int g=0;
int start_x=round(width/4);
int start_y=round(height/4);

int end_x=width;
int end_y=height;
//end.z=0;
cout<<"not equal"<<endl;
Mat obstacles=Mat::zeros(width,height,CV_8UC1);
cout<<"not equal"<<endl;
Mat tick_grid = Mat::zeros(width,height,CV_8UC1);
Mat gvalue = Mat::zeros(width,height,CV_64F);
Mat cost_function = Mat::zeros(width,height,CV_64F);
tick_grid=local_map;
//tick_grid.at<uchar>(start.x,start.y)=1;

for(int s=0;s<width;s++)
   {
     for(int q=0;q<height;q++)
     {
       cost_function.at<float>(s,q) = sqrt((s-end_x)*(s-end_x)+(q-end_y)*(q-end_y));
     }
   }
   cout<<"going to search"<<endl;
search(gvalue,tick_grid,start_x,start_y,width,height,g,reached_end,end_x,end_y,cost_function);
cout<<"steps=";
for(int s=0;s<width;s++)
   {
     for(int q=0;q<height;q++)
     {
       obstacles.at<uchar>(s,q) = tick_grid.at<uchar>(s,q)*255;
     }
   }
int k=gvalue.at<float>(end_x,end_y);
cout<<k;
//figure(2);
imshow("path",obstacles);
//cout<<gvalue;

waitKey(1);
//
}
//void chatterCallback(const polysync_ros_bridge::ps_radar_targets_msg::ConstPtr& msg)
void chatterCallback(nav_msgs::OccupancyGrid msg)
{
nav_msgs::OccupancyGrid m;
geometry_msgs::Quaternion z;
/**/
int height=msg.info.height;int width=msg.info.width;
    m.header.stamp = ros::Time::now();
    m.header.frame_id = "/path_map";
    m.info.resolution = msg.info.resolution;
    m.info.origin.position.x = 0.0;
    m.info.origin.position.y = 0.0;
    m.info.origin.position.z = 0.0;
    m.info.origin.orientation.x = 0.0;
    m.info.origin.orientation.y = 0.0;
    m.info.origin.orientation.z = 0.0;
    m.info.origin.orientation.w = 1.0;
    m.info.width = width;
    m.info.height = height;
    m.info.map_load_time = ros::Time::now();
  for(int i=0;i<height;i++)//initialising map
    {
       for(int j=0;j<width;j++)
        {
           m.data.push_back(0);
        }
    }
    /**/
ROS_INFO("I heard1:");
 z.x=0;
z.y=0;
z.z=0;
z.w=0;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
/*    float resolution=0.09;
    m.info.resolution=resolution;
m.info.origin.position.x=0 ;
m.info.origin.position.y=0 ;
m.info.origin.orientation=z;
int width=1000;
int height=1000;
float a[63];
float b[63];
m.info.width=width;
m.info.height=height;
ROS_INFO("I heard2:");
for(int i=0;i<63;i++)
{
a[i]=msg->targets[i].position[0];
b[i]=msg->targets[i].position[1];
}
ROS_INFO("I heard positions:%f,%f",a[0],b[0]);
for(int u=0;u<63;u++)
{
a[u]=a[u]/resolution;
b[u]=b[u]/resolution;
}
ROS_INFO("I heard6:%f,%f",a[0],a[0]);
int c[63];
int d[63];
for(int u=0;u<63;u++)
{
c[u]=round(a[u]);
d[u]=round(b[u]);
}
ROS_INFO("I heard3:%d,%d",c[0],d[0]);
//a and b is in meters
//a=a/resolution
for(int j=0;j<width;j++)
{
for(int k=0;k<height;k++)
{
m.data.push_back(0);
}
}
ROS_INFO("I heard5:");
for(int u=0;u<63;u++)
{
if(a[u]<width/2&&b[u]<height/2)
{
m.data[width*(height/2)+width/2-c[u]*height-d[u]]=100;
//m.data[b*500+a]=100;
}
}*/

/*
*/
   for(int i=0;i<height;i++)//initialising map
    {
       for(int j=0;j<width;j++)
        {
           m.data[i*width+j]=msg.data[i*width+j];
        }
    }

Mat local_map = Mat::zeros(width,height,CV_8UC1);
   for(int s=0;s<height;s++)
   {
     for(int q=0;q<width;q++)
     { 
      int i_= height - s;//i=x,j=y;
       local_map.at<uchar>(i_,q) = m.data[s*width+q]*255;
     }
   }
   cout<<"core dumped"<<endl;

path_planning(local_map);
//Mat large_image;'
//figure(1);
imshow("large_image",local_map);
//resize(local_map,large_image,cvSize(1000,1000));
 //  imshow("large_image",large_image);
waitKey(1);

gridmap_pub.publish(m);    
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe<nav_msgs::OccupancyGrid>("/scan/local_map",25,chatterCallback);
//ros::Subscriber sub2 = n.subscribe("chatter",1000, chatterCallback );
gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("astar", 10);
 int count = 0;
ros::Rate loop_rate(20);

//ROS_INFO("I heard4");
  while (ros::ok())
  {
  ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
 
 


  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  

  return 0;
}


