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
//radar libraries
#include <polysync_ros_bridge/ps_msg_header.h>
#include <polysync_ros_bridge/ps_sensor_descriptor.h>
#include <polysync_ros_bridge/ps_radar_targets_msg.h>
using namespace std;
using namespace cv;

const float res=0.05;             //resolution of the grid map in m/cell
const int num_values = 63;       //no of targets given by radar at once
const int radius_in_cm = 60;     //radius around the object for blown map
const int side_in_cm = 30;
const int base_radar_dist_in_cm = 50;  //distance between radar and base

const int width = 500;           //map size
const int height = 500;
const int range = 200;
int l_height,l_width;
float l_res;

nav_msgs::OccupancyGrid local_map;  //occupancy grid to copy the local map

ros::Publisher radar_map_pub,fused_map_pub,blown_fused_map_pub;

void localmap_Callback(nav_msgs::OccupancyGrid msg)
{
   int i,j;
   l_res=msg.info.resolution;
   l_width = msg.info.width;
   l_height = msg.info.height;
   local_map.header.stamp = ros::Time::now();
   local_map.header.frame_id = "/local_map1";
   local_map.info.resolution = l_res;
   local_map.info.origin.position.x = 0.0;
   local_map.info.origin.position.y = 0.0;
   local_map.info.origin.position.z = 0.0;
   local_map.info.origin.orientation.x = 0.0;
   local_map.info.origin.orientation.y = 0.0;
   local_map.info.origin.orientation.z = 0.0;
   local_map.info.origin.orientation.w = 1.0;
   local_map.info.width = l_width;
   local_map.info.height = l_height;
   local_map.info.map_load_time = ros::Time::now();
   //cout<<l_width<<" "<<l_height<<endl;
   for(i=0;i<l_height;i++)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         local_map.data.push_back(0);
      }
   }
   
   for(i=0;i<l_height;i++)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         local_map.data[i*l_width+j] = msg.data[i*l_width+j];
      }
   }
    cv::Mat local_map_cv = cv::Mat::zeros(l_height,l_width,CV_8UC1);
    for(i=0;i<l_height;i++)
    {
      for(j=0;j<l_width;j++)
      {
        int i_= height - i;
        local_map_cv.at<uchar>(i,j) = local_map.data[i_*l_width+j]*0.5*255;
        //blown_local_map.at<uchar>(i,j) = blown_fused_map.data[i_*width+j]*0.5*255;
        //local_map.at<Vec3b>(i,j)[1] = map.data[i*width+j]*255;
        //local_map.at<Vec3b>(i,j)[2] = map.data[i*width+j]*255;
      }
    }
    imshow("local_map",local_map_cv);
    cv::waitKey(1);    
}
void radarCallback(const polysync_ros_bridge::ps_radar_targets_msg::ConstPtr& msg)
{
    float radius_in_cells;
    float base_radar_dist_in_cells = (base_radar_dist_in_cm*0.01)/res;
    int i,j,k,l;
    nav_msgs::OccupancyGrid radar_map;
    radar_map.header.stamp = ros::Time::now();
    radar_map.header.frame_id = "/radar";
    radar_map.info.resolution = res;
    radar_map.info.origin.position.x = 0.0;
    radar_map.info.origin.position.y = 0.0;
    radar_map.info.origin.position.z = 0.0;
    radar_map.info.origin.orientation.x = 0.0;
    radar_map.info.origin.orientation.y = 0.0;
    radar_map.info.origin.orientation.z = 0.0;
    radar_map.info.origin.orientation.w = 1.0;
    radar_map.info.width = width;
    radar_map.info.height = height;
    radar_map.info.map_load_time = ros::Time::now();

    nav_msgs::OccupancyGrid fused_map;
    fused_map.header.stamp = ros::Time::now();
    fused_map.header.frame_id = "/fused";
    fused_map.info.resolution = res;
    fused_map.info.origin.position.x = 0.0;
    fused_map.info.origin.position.y = 0.0;
    fused_map.info.origin.position.z = 0.0;
    fused_map.info.origin.orientation.x = 0.0;
    fused_map.info.origin.orientation.y = 0.0;
    fused_map.info.origin.orientation.z = 0.0;
    fused_map.info.origin.orientation.w = 1.0;
    fused_map.info.width = width;
    fused_map.info.height = height;
    fused_map.info.map_load_time = ros::Time::now();

    nav_msgs::OccupancyGrid blown_fused_map;
    fused_map.header.stamp = ros::Time::now();
    fused_map.header.frame_id = "/blown_fused";
    fused_map.info.resolution = res;
    fused_map.info.origin.position.x = 0.0;
    fused_map.info.origin.position.y = 0.0;
    fused_map.info.origin.position.z = 0.0;
    fused_map.info.origin.orientation.x = 0.0;
    fused_map.info.origin.orientation.y = 0.0;
    fused_map.info.origin.orientation.z = 0.0;
    fused_map.info.origin.orientation.w = 1.0;
    fused_map.info.width = width;
    fused_map.info.height = height;
    fused_map.info.map_load_time = ros::Time::now();

    for(i=0;i<height;i++)//initialising map
    {
       for(j=0;j<width;j++)
        {
           fused_map.data.push_back(0);
           radar_map.data.push_back(0);
           blown_fused_map.data.push_back(0);
        }
    }
    //throwing the radar data to the grid
    for(i=0;i<num_values;i++)
    {
       int x_coord = round((msg->targets[i].position[0]+base_radar_dist_in_cm*0.01)/res);
       int y_coord = round(msg->targets[i].position[1]/res);
       /*if(x_coord==0 && y_coord==0)
       {
         cout<<i<<" "<<msg->ranges[i];
       }*/
       if(x_coord > 0 && x_coord <= range)//limiting obstacle detection to only 10m
        {
          if(y_coord <= range && y_coord >= -range)
          {
             if(!(x_coord==base_radar_dist_in_cells && y_coord==0))
             {
                fused_map.data[x_coord*width+y_coord+width/2] = 3;
                radar_map.data[x_coord*width+y_coord+width/2] = 3;
             }    
          } 
        }
    }
    //updating the grid with the local_map
    for(i=0;i<l_height;i++)//adding lane data 
    { 
       for(j=0;j<l_width;j++)
       {  //cout<<round(i*(l_res/res)*width+j*(l_res/res)+width/2-l_width*(l_res/res)/2)<<endl;
            int q=round(i*l_res/res);
            //cout<<q<<endl;
          if(local_map.data[(i)*l_width+j]==2)
            {
            //fused_map.data[i*width+j+width/2-l_width/2] = 2;
            fused_map.data[round(q*width+(j-l_width/2)*(l_res/res)+width/2)] = 2;
            }
          if(local_map.data[(i)*l_width+j]==1)
            {
            //fused_map.data[i*width+j+width/2-l_width/2] = 1;
            fused_map.data[round(q*width+(j-l_width/2)*(l_res/res)+width/2)] = 1;
            }  
       }
    }
    for(i=0;i<height;i++)//creating blown map
    {
       for(j=0;j<width;j++)
       {
          if(fused_map.data[i*width+j]==3)//blowing obstacle data
          {
            if(i<base_radar_dist_in_cells)
            radius_in_cells = (side_in_cm*0.01)/res;
            else
            radius_in_cells = (radius_in_cm*0.01)/res;
 
            for(k=-1*radius_in_cells;k<=radius_in_cells;k++)
            {
              for(l=-1*radius_in_cells;l<=radius_in_cells;l++)
              {
                if((k*k + l*l) < radius_in_cells*radius_in_cells)
                {
                  int x = i + k;
                  int y = j + l - width/2;
                  if(x >= 0 && x <= range)
                  {
                   if(y <= range && y >= -range)
                   {
                    blown_fused_map.data[x*width+y+width/2] = 3;
                   }
                  }
                }
              }
            }
          }
          if(fused_map.data[i*width+j]==1)//blowing obstacle data
          {
            if(i<base_radar_dist_in_cells)
            radius_in_cells = (side_in_cm*0.01)/res;
            else
            radius_in_cells = (radius_in_cm*0.01)/res;
 
            for(k=-1*radius_in_cells;k<=radius_in_cells;k++)
            {
              for(l=-1*radius_in_cells;l<=radius_in_cells;l++)
              {
                if((k*k + l*l) < radius_in_cells*radius_in_cells)
                {
                  int x = i + k;
                  int y = j + l - width/2;
                  if(x >= 0 && x <= range)
                  {
                   if(y <= range && y >= -range)
                   {
                    blown_fused_map.data[x*width+y+width/2] = 1;
                   }
                  }
                }
              }
            }
          }
        }
    }
    radar_map_pub.publish(radar_map);
    fused_map_pub.publish(fused_map);
    blown_fused_map_pub.publish(blown_fused_map);
    cv::Mat fused_map_cv = cv::Mat::zeros(height,width,CV_8UC1);
    cv::Mat radar_map_cv = cv::Mat::zeros(height,width,CV_8UC1);
    cv::Mat blown_fused_map_cv = cv::Mat::zeros(height,width,CV_8UC3);
    for(i=0;i<height;i++)
    {
      for(j=0;j<width;j++)
      {
        int i_= height - i;
        fused_map_cv.at<uchar>(i,j) = fused_map.data[i_*width+j]*0.5*255;
        radar_map_cv.at<uchar>(i,j) = radar_map.data[i_*width+j]*0.5*255;
        //blown_fused_map_cv.at<uchar>(i,j) = blown_fused_map.data[i_*width+j]*0.5*255;
        if(blown_fused_map.data[i_*width+j]==3)
        {
        blown_fused_map_cv.at<Vec3b>(i,j)[1] = blown_fused_map.data[i_*width+j]*255;
        blown_fused_map_cv.at<Vec3b>(i,j)[2] = blown_fused_map.data[i_*width+j]*255;
        }
        if(blown_fused_map.data[i_*width+j]==1)
        {
        blown_fused_map_cv.at<Vec3b>(i,j)[0] = blown_fused_map.data[i_*width+j]*55;
        blown_fused_map_cv.at<Vec3b>(i,j)[2] = blown_fused_map.data[i_*width+j]*55;
        }
      }
    }
    for(int u=0;u<num_values;u++)
    { int x_coord = round((msg->targets[u].position[0]+base_radar_dist_in_cm*0.01)/res);
      int y_coord = round(msg->targets[u].position[1]/res);
      if(x_coord<width&&y_coord<height/2)
        { 
          if(abs(msg->targets[u].velocity[0])<1000&&abs(msg->targets[u].velocity[1])<1000)
          {
            int y=msg->targets[u].track_status.value;
            stringstream ss;
            ss << y;
            string str = ss.str();
            putText(radar_map_cv,str,cvPoint(width/2+y_coord,height-x_coord), FONT_HERSHEY_DUPLEX, 1, cvScalar(200), 1, 8, false );
            putText(fused_map_cv,str,cvPoint(width/2+y_coord,height-x_coord), FONT_HERSHEY_DUPLEX, 1, cvScalar(200), 1, 8, false );
            putText(blown_fused_map_cv,str,cvPoint(width/2+y_coord,height-x_coord), FONT_HERSHEY_DUPLEX, 1, cvScalar(200), 1, 8, false );
          }
        }
    }
    imshow("fused_map",fused_map_cv);
    imshow("radar_map",radar_map_cv);
    imshow("blown_fused_map",blown_fused_map_cv);
    cv::waitKey(1);
    fused_map.data.clear();
    radar_map.data.clear();
    blown_fused_map.data.clear();  
}
int main(int argc, char **argv)
{
	ros::init(argc,argv,"fused_map");
	ros::NodeHandle n,n1,n2,n3,n4;
	ros::Subscriber local_map_sub=n1.subscribe<nav_msgs::OccupancyGrid>("scan/local_map",1,localmap_Callback);
	ros::Subscriber radar_sub = n.subscribe("polysync/ps_radar_targets_msg",1, radarCallback );
    radar_map_pub=n2.advertise<nav_msgs::OccupancyGrid>("radar_grid",1);
    fused_map_pub=n3.advertise<nav_msgs::OccupancyGrid>("fused_grid",1);
    blown_fused_map_pub=n4.advertise<nav_msgs::OccupancyGrid>("blown_fused_grid",1);

    namedWindow("blown_fused_map",CV_WINDOW_NORMAL);
    namedWindow("radar_map",CV_WINDOW_NORMAL);
    namedWindow("fused_map",CV_WINDOW_NORMAL);

    ros::Rate loop_rate(15.0);

    while(ros::ok())
    {
     ros::spinOnce();//check for incoming messages
     loop_rate.sleep(); 
    }
    return 0;
}