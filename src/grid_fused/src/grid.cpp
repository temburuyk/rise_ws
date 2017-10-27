#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <polysync_ros_bridge/ps_msg_header.h>
#include <polysync_ros_bridge/ps_sensor_descriptor.h>
#include <polysync_ros_bridge/ps_radar_targets_msg.h>
#include <geometry_msgs/Quaternion.h>
/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

nav_msgs::OccupancyGrid m;
geometry_msgs::Quaternion z;

int i=0;
void chatterCallback(const polysync_ros_bridge::ps_radar_targets_msg::ConstPtr& msg)
{
ROS_INFO("I heard1:");
 z.x=0;
z.y=0;
z.z=0;
z.w=0;
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    
    m.info.resolution=0.5;
m.info.origin.position.x=0 ;
m.info.origin.position.y=0 ;
m.info.origin.orientation=z;
m.info.width=100;
m.info.height=100;
ROS_INFO("I heard2:");
float a=msg->targets[0].position[0];
float b=msg->targets[0].position[1];
ROS_INFO("I heard positions:%f,%f",a,b);
a=a*20;
b=b*20;
int c=a;
int d=b;
ROS_INFO("I heard3:%d,%d",c,d);
//a and b is in meters
//a=a/resolution
for(int j=0;j<100;j++)
{
for(int k=0;k<100;k++)
{
m.data.push_back(0);
}
}
if(a<200&&b<200)
{
m.data[100*50+50+b*100+a]=100;
ROS_INFO("I heard5:");
}
i++;
 

   
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
ros::Publisher gridmap_pub = n.advertise<nav_msgs::OccupancyGrid>("gridmap", 1);
 int count = 0;
ros::Rate loop_rate(20);

ROS_INFO("I heard4");
  while (ros::ok())
  {
  ros::spinOnce();
gridmap_pub.publish(m);
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

