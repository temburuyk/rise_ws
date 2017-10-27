//ros libraries
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/Int8MultiArray.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/GetMap.h"
//opencv libraries
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

//c++ libraries
#include <iostream>
#include <vector>
#include <utility>
#include <math.h>
using namespace std;
using namespace cv;

const float res = 0.25 ;         //angular resolution of lidar in degrees
const int num_values = 1080;     //no. of rays emitted by the lidar

const int radius_in_cm = 60;     //radius around on object to be in safe limit for path-planning
const int lane_in_cm = 30;       //radius around lane for path planning
const int side_in_cm = 30;
const int base_lidar_dist = 50;   //distance from vehicle centre to lidar in cm(only in y-direction)

const int cam_lidar_dist = 50;    //distance from camera to lidar in cm
float map_res = 0.25;//map resolution(m/cell)
const int width = 40/map_res;//map size
const int height = 30/map_res;
const int range = 10/map_res;//10m in cells
int l_width,l_height;//lane map size          

int count1; 
ros::Publisher map_pub1,blown_map_pub,occ_map_pub;
std_msgs::Int8MultiArray lane_map;
nav_msgs::Path path;

cv::Mat path_local_map = cv::Mat::zeros(height,width,CV_8UC3);

pair<float,float> convToCart(int i,float r) //convert from polar to cartesian co-ordinates
{
   float ang = i*0.25*(3.14/180);
   float x = r*cos(ang);
   float y = -1*r*sin(ang);
   return make_pair(x,y);
}

void laneCallback(nav_msgs::OccupancyGrid msg)
{
   int i,j;
   l_width = msg.info.width;
   l_height = msg.info.height;
   //cout<<l_width<<" "<<l_height<<endl;
   for(i=0;i<l_height;i++)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         lane_map.data.push_back(0);
      }
   }
   
   for(i=l_height-1;i>=0;i--)//initialising map
   {
      for(j=0;j<l_width;j++)
      {
         int i1;
         i1=l_height-1-i;

         lane_map.data[i1*l_width+j] = msg.data[i*l_width+j];
      }
   }    
}

void lidarCallback(sensor_msgs::LaserScan msg)
{

   float radius_in_cells;
   float minor_axis_in_cells = (lane_in_cm*0.01)/map_res;//elliptically blowing lanes
   float major_axis_in_cells = minor_axis_in_cells + 3;
   float base_lidar_dist_in_cells = (base_lidar_dist*0.01)/map_res;
   float base_cam_dist_in_cells = ((cam_lidar_dist - base_lidar_dist)*0.01)/map_res;
   //cout<<base_lidar_dist_in_cells<<endl;
   int i,j,k,l;

   nav_msgs::OccupancyGrid grid_map;
   grid_map.header.stamp = ros::Time::now();
   grid_map.header.frame_id = "/map";
   grid_map.info.resolution = map_res;
   grid_map.info.origin.position.x = 0.0;
   grid_map.info.origin.position.y = 0.0;
   grid_map.info.origin.position.z = 0.0;
   grid_map.info.origin.orientation.x = 0.0;
   grid_map.info.origin.orientation.y = 0.0;
   grid_map.info.origin.orientation.z = 0.0;
   grid_map.info.origin.orientation.w = 1.0;
   grid_map.info.width = width;
   grid_map.info.height = height;
   grid_map.info.map_load_time = ros::Time::now();

   nav_msgs::OccupancyGrid blown_grid_map;
   blown_grid_map.header.stamp = ros::Time::now();
   blown_grid_map.header.frame_id = "/map";
   blown_grid_map.info.resolution = map_res;
   blown_grid_map.info.origin.position.x = 0.0;
   blown_grid_map.info.origin.position.y = 0.0;
   blown_grid_map.info.origin.position.z = 0.0;
   blown_grid_map.info.origin.orientation.x = 0.0;
   blown_grid_map.info.origin.orientation.y = 0.0;
   blown_grid_map.info.origin.orientation.z = 0.0;
   blown_grid_map.info.origin.orientation.w = 1.0;
   blown_grid_map.info.width = width;
   blown_grid_map.info.height = height;
   blown_grid_map.info.map_load_time = ros::Time::now();
   
   nav_msgs::GetMap::Response dummy_map;
   dummy_map.map.header.stamp = ros::Time::now();
   dummy_map.map.header.frame_id = "/map";
   dummy_map.map.info.resolution = 0.25;
   dummy_map.map.info.origin.position.x = 0.0;
   dummy_map.map.info.origin.position.y = 0.0;
   dummy_map.map.info.origin.position.z = 0.0;
   dummy_map.map.info.origin.orientation.x = 0.0;
   dummy_map.map.info.origin.orientation.y = 0.0;
   dummy_map.map.info.origin.orientation.z = 0.0;
   dummy_map.map.info.origin.orientation.w = 1.0;
   dummy_map.map.info.width = 160;
   dummy_map.map.info.height = 120;
   dummy_map.map.info.map_load_time = ros::Time::now();

   vector< pair<float,float> > cartesian(num_values,make_pair(0,0));
   for(i=0;i<num_values;i++)
   {
      cartesian[i] = convToCart((i-num_values/2),msg.ranges[i-180]);
      cartesian[i].first += base_lidar_dist*0.01;
   }
   //float min=20;
   /*for(i=0;i<num_values;i++)
   {
      if(msg.ranges[i]<min)
      {min=msg.ranges[i];}
      //if(msg.ranges[i]==0.0)
      //{cout<<i;}
   }*/
   //cout<<"min="<<min<<",straight="<<msg.ranges[540]<<endl;
   //int def=round(cartesian[540].first/map_res);
   //cout<<def;
   //std_msgs::Int8MultiArray map;
   for(i=0;i<height;i++)//initialising map
   {
      for(j=0;j<width;j++)
      {
         grid_map.data.push_back(0);
         blown_grid_map.data.push_back(0);
         dummy_map.map.data.push_back(0);
      }
   }
   //cout<<"hi"<<endl;
   for(i=0;i<num_values;i++)
   {
      int x_coord = round(cartesian[i].first/map_res);
      int y_coord = round(cartesian[i].second/map_res);
      /*if(x_coord==0 && y_coord==0)
      {
        cout<<i<<" "<<msg.ranges[i];
      }*/
      if(x_coord > 1 && x_coord <= range)//limiting obstacle detection to only 10m
      {
         if(y_coord <= range && y_coord >= -range)
         {
            if(!(x_coord==base_lidar_dist_in_cells && y_coord==0))
            grid_map.data[x_coord*width+y_coord+width/2] = 100;
         } 
      }
   }
   //int x_ = round((cartesian[540].first + base_lidar_dist)/map_res);
   //int y_ = round(cartesian[540].second/map_res);
   //int a=blown_grid_map.data[400];
   //int b=grid_map.data[400];
   //cout<<a<<" "<<b<<endl;
   //grid_map.data = map.data;
   
   for(i=0;i<l_height;i++)//adding lane data 
   {
      for(j=0;j<l_width;j++)
      {
         if(lane_map.data[(i+base_cam_dist_in_cells)*l_width+j]==2)
         grid_map.data[i*width+j+width/2-l_width/2] = 2;
      }
   }

   for(i=0;i<height;i++)//creating blown map
   {
      for(j=0;j<width;j++)
      {
         if(grid_map.data[i*width+j]==1)//blowing obstacle data
         {
            if(i<base_lidar_dist_in_cells)
            radius_in_cells = (side_in_cm*0.01)/map_res;
            else
            radius_in_cells = (radius_in_cm*0.01)/map_res;

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
                           blown_grid_map.data[x*width+y+width/2] = 1;
                        }
                     }
                  }
               }
            }
         }
         
         if(grid_map.data[i*width+j]==2)//blowing lane data
         {
            for(k=-1*major_axis_in_cells;k<=major_axis_in_cells;k++)
            {
               for(l=-1*minor_axis_in_cells;l<=minor_axis_in_cells;l++)
               {
                  if(((minor_axis_in_cells*minor_axis_in_cells)*(k*k) + (major_axis_in_cells*major_axis_in_cells)*(l*l)) < (minor_axis_in_cells*minor_axis_in_cells)*(major_axis_in_cells*major_axis_in_cells))//inside ellipse condition
                  {
                     int x = i + k;
                     int y = j + l - width/2;
                     if(x >= 0 && x <= range)
                     {
                        if(y <= range && y >= -range)
                        {
                           blown_grid_map.data[x*width+y+width/2] = 2;
                        }
                     }
                  }
               }
            }
         }

      }
   }   
/*for(j=0;j<80;j++)
     {
       dummy_map.map.data[30*width+j+width/2] = 100;
     }
   for(i=0;i<30;i++)
   {
     
       dummy_map.map.data[i*width+10+width/2] = 100;
   }
     for(j=0;j<35;j++)
     {
       dummy_map.map.data[10*width+j] = 100;
     }

     for(i=20;i<30;i++)
     {
       dummy_map.map.data[i*width+30] = 100;
     }*/
   //map_pub1.publish(grid_map);
   //occ_map_pub.publish(grid_map);
   map_pub1.publish(dummy_map.map);
   //occ_map_pub.publish(dummy_map);
   blown_map_pub.publish(blown_grid_map);
   cv::Mat local_map = cv::Mat::zeros(height,width,CV_8UC1);
   cv::Mat blown_local_map = cv::Mat::zeros(height,width,CV_8UC1);
   cv::Mat dummy_map_mat = cv::Mat::zeros(height,width,CV_8UC1);
   for(i=0;i<height;i++)
   {
     for(j=0;j<width;j++)
     {
       int i_= height - i;
       local_map.at<uchar>(i,j) = grid_map.data[i_*width+j]*0.5*255;
       blown_local_map.at<uchar>(i,j) = blown_grid_map.data[i_*width+j]*0.5*255;
       path_local_map.at<Vec3b>(i,j)[1] = local_map.at<uchar>(i,j);
       dummy_map_mat.at<uchar>(i,j) = dummy_map.map.data[i_*width+j]*0.5*255;
       //local_map.at<Vec3b>(i,j)[1] = map.data[i*width+j]*255;
       //local_map.at<Vec3b>(i,j)[2] = map.data[i*width+j]*255;
     }
   }
   imshow("local_map",local_map);
   imshow("dummy_map_mat",dummy_map_mat);
   imshow("blown_local_map",blown_local_map);
   imshow("path_local_map",path_local_map);
   cv::waitKey(1);
   lane_map.data.clear();
   grid_map.data.clear();
   blown_grid_map.data.clear();
   dummy_map.map.data.clear();
}

void path_show(const visualization_msgs::MarkerArray::ConstPtr& msg1){
//cout<<"entered path_show"<<endl;

geometry_msgs::PoseStamped y;
cout<<"defined path"<<endl;
//list<geometry_msgs::PoseStamped>::iterator it = msg1->poses.begin();
//if (msg1->poses.begin()!=0)

   //cout<<"msg exits"<<endl;
   cout<<msg1->markers[20].pose.position.y<<endl;

count1++;
}
int main(int argc,char** argv)
{
   count1=0;
   ros::init(argc,argv,"local_map");
   ros::NodeHandle n;
   //ros::Subscriber path_sub=n.subscribe<nav_msgs::Path>("/path", 1,path_show);
   //ros::Subscriber path_sub=n.subscribe<visualization_msgs::MarkerArray>("/pathNodes", 1,path_show);
   ros::Subscriber lane_sub=n.subscribe<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid",1,laneCallback);
   ros::Subscriber lidar_sub=n.subscribe<sensor_msgs::LaserScan>("scan",1,lidarCallback);
   //map_pub=n2.advertise<nav_msgs::OccupancyGrid>("scan/local_map",1);
   blown_map_pub=n.advertise<nav_msgs::OccupancyGrid>("scan/blown_local_map",1);
   map_pub1 = n.advertise<nav_msgs::OccupancyGrid>("/scan/local_map",1);
   //occ_map_pub=n.advertise<nav_msgs::OccupancyGrid>("occ_map",1);

   cv::namedWindow("path_local_map",CV_WINDOW_NORMAL);
   cv::namedWindow("dummy_map_mat",CV_WINDOW_NORMAL);
   cv::namedWindow("local_map",CV_WINDOW_NORMAL);
   cv::namedWindow("blown_local_map",CV_WINDOW_NORMAL);

   //ros::Rate loop_rate(100);
//ros::spin();
   while(ros::ok())
   {
     ros::spinOnce();//check for incoming messages
     
   }
   return 0; 
}
