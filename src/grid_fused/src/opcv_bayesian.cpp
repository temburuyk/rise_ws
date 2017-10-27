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

#include <polysync_ros_bridge/ps_msg_header.h>
#include <polysync_ros_bridge/ps_sensor_descriptor.h>
#include <polysync_ros_bridge/ps_radar_targets_msg.h>
#include "bayesian.h"

#define GRIDROWS 70
#define GRIDCOLS 70

using namespace std;
using namespace cv;

ros::Publisher gridmap_pub1;

int occupancy = 0;
int once=0;
void displayOccupancyGrid(vector<vector<bof::Cell> >& occGrid) {

    Mat mapImg = Mat::zeros(GRIDROWS, GRIDCOLS, CV_8UC3);

    for (int i = 0; i < GRIDROWS; ++i) {
        for (int j = 0; j < GRIDCOLS; ++j) {
            mapImg.at<Vec3b>(i, j)[0] = occGrid[i][j].getOccupiedProbability() * 255;
            mapImg.at<Vec3b>(i, j)[1] = occGrid[i][j].getOccupiedProbability() * 255;
            mapImg.at<Vec3b>(i, j)[2] = occGrid[i][j].getOccupiedProbability() * 255;
        }
    }

    //    while (waitKey(30) != 27) { // wait for ESC key press
    //        imshow("occupancyGrid", mapImg);
    //    }
    waitKey(1);
    imshow("occupancyGrid", mapImg);
}

void once_execute(vector<vector<bof::Cell> > &occGrid)
{

 bof::VelocityDistribution xVelDist(-GRIDROWS/2, GRIDROWS, 1, 0);
    bof::VelocityDistribution yVelDist(-GRIDCOLS/2, GRIDCOLS, 1, 0);
  for (int y = 0; y < GRIDCOLS;++y) {
        vector<bof::Cell> occRow;
        for (int x = 0; x <GRIDROWS; ++x) {
            
            bof::Cell cell(xVelDist, yVelDist, occupancy, x,y );
             occRow.push_back(cell);
            }
            occGrid.push_back(occRow);
        }


}
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */


void chatterCallback(const polysync_ros_bridge::ps_radar_targets_msg::ConstPtr& msg)
{
  cout<<"i heard8"<<endl;
nav_msgs::OccupancyGrid m;
geometry_msgs::Quaternion z;
vector<vector<bof::Cell> > occGrid;
  once_execute(occGrid);
//ROS_INFO("I heard1:");
 z.x=0;
z.y=0;
z.z=0;
z.w=0;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    float resolution=0.15;
    m.info.resolution=resolution;
m.info.origin.position.x=0 ;
m.info.origin.position.y=0 ;
m.info.origin.orientation=z;
int width=GRIDROWS;
int height=GRIDCOLS;
float a[63];
float b[63];
int nooftargets=0;
m.info.width=width;
m.info.height=height;
ROS_INFO("I heard2:");
int targetno[63];

for(int i=0;i<64;i++)
{
  if(msg->targets[i].velocity[0]>0.015&&msg->targets[i].velocity[1]>0.015)
  {
  float r=msg->targets[i].velocity[0];
    float t=msg->targets[i].velocity[1];
    float y=msg->targets[i].track_status.value;
    float q=msg->targets[i].position[0];
float w=msg->targets[i].position[1];
    cout<<r<<" "<<t<<" "<<y<<" "<<q<<" "<<w<<endl;
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
cout<<"i heard7";
for(int j=0;j<width;j++)
{
for(int k=0;k<height;k++)
{
m.data.push_back(0);
}
}
ROS_INFO("I heard5:");
for(int u=0;u<nooftargets;u++)
{
if(a[u]<width/2&&b[u]<height/2)
{
m.data[width*(height/2)+width/2-c[u]*height-d[u]]=100;
float r=msg->targets[targetno[u]].velocity[0];
    float t=msg->targets[targetno[u]].velocity[1];
r=round(r/resolution);
t=round(t/resolution);
 bof::VelocityDistribution xVelDist(-GRIDROWS/2, GRIDROWS, 1, 0);
    bof::VelocityDistribution yVelDist(-GRIDCOLS/2, GRIDCOLS, 1, 0);
    yVelDist.setVelocityProbability(-r, 1);
    xVelDist.setVelocityProbability(-t, 1);

     occupancy = 1;
     bof::Cell cell(xVelDist, yVelDist, occupancy, width/2-d[u],height/2-c[u] );
     // Initialize occupancy grid 
        occGrid[width/2-d[u]][height/2-c[u]]=cell;
    }
//m.data[b*500+a]=100;
}
Mat local_map = Mat::zeros(width,height,CV_8UC1);
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
{  int y=msg->targets[targetno[u]].track_status.value;
stringstream ss;
ss << y;
string str = ss.str();
   putText(local_map,str,cvPoint(width/2-d[u],height/2-c[u]), FONT_HERSHEY_DUPLEX, 1, cvScalar(200), 1, 8, false );
}
}
Mat large_image;
imshow("large_image",local_map);
//resize(local_map,large_image,cvSize(1000,1000));
 //  imshow("large_image",large_image);
waitKey(1);
gridmap_pub1.publish(m); 
namedWindow("occupancyGrid", CV_WINDOW_NORMAL);
        /* Update occupancy grid */
        vector<vector<bof::Cell> > prevOccGrid = occGrid;

      /*  for (int i = 0; i < GRIDROWS; ++i) {
            for (int j = 0; j < GRIDCOLS; ++j) {
                occGrid[i][j].updateDistributions(prevOccGrid);
                // cout << "[" << i << "][" << j << "]: ";
                // occGrid[i][j].toString();
            }
        }*/
        for(int u=0;u<nooftargets;u++)
   {
      if(a[u]<width/2&&b[u]<height/2)
      {
      m.data[width*(height/2)+width/2-c[u]*height-d[u]]=100;
      float r=msg->targets[targetno[u]].velocity[0];
    float t=msg->targets[targetno[u]].velocity[1];
      r=round(r/resolution);
      t=round(t/resolution);
        bof::VelocityDistribution xVelDist(-GRIDROWS/2, GRIDROWS, 1, 0);
        bof::VelocityDistribution yVelDist(-GRIDCOLS/2, GRIDCOLS, 1, 0);
        yVelDist.setVelocityProbability(-r, 1);
        xVelDist.setVelocityProbability(-t, 1);
         occGrid[width/2-d[u]][height/2-c[u]].updateDistributions(prevOccGrid,t,r);
    }
//m.data[b*500+a]=100;
  }
//         displayOccupancyGrid(occGrid);  
}


int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "grid");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */
  ros::Subscriber sub = n.subscribe("polysync/ps_radar_targets_msg",1, chatterCallback );
//ros::Subscriber sub2 = n.subscribe("chatter",1000, chatterCallback );
gridmap_pub1 = n.advertise<nav_msgs::OccupancyGrid>("gridmap1", 1);


 int count = 0;
ros::Rate loop_rate(50);

ROS_INFO("I heard4");
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

