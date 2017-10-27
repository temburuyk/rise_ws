#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

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
  ros::init(argc, argv, "pose");

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
//ros::Subscriber sub2 = n.subscribe("chatter",1000, chatterCallback );



  geometry_msgs::PoseStamped x1;
  geometry_msgs::PoseWithCovarianceStamped x2;

  x1.pose.position.x=20;
  x1.pose.position.y=20;
  x1.pose.orientation.x=0;
x1.pose.orientation.y=1;
x1.pose.orientation.z=0;
x1.pose.orientation.w=0;

 x2.pose.pose.position.x=20;
 x2.pose.pose.position.y=10;
 x2.pose.pose.orientation.x=0;
 x2.pose.pose.orientation.y=0;
 x2.pose.pose.orientation.z=0;
 x2.pose.pose.orientation.w=1;
for(int i=0;i<36;i++)
{
  x2.pose.covariance[i]=0;
}

ros::Publisher pose_pub1 = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose", 1);
ros::Publisher pose_pub2 = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1);

 int count = 0;
ros::Rate loop_rate(100);

ROS_INFO("I heard4");
  while (ros::ok())
  {
    
  ros::spinOnce();
pose_pub1.publish(x2);
pose_pub2.publish(x1);
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