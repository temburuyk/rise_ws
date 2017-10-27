
//Includes all the headers necessary to use the most common public pieces of the ROS system.
#include <ros/ros.h>
//Use image_transport for publishing and subscribing to images in ROS
#include <image_transport/image_transport.h>
//Use cv_bridge to convert between ROS and OpenCV Image formats
#include <cv_bridge/cv_bridge.h>
//Include some useful constants for image encoding. Refer to: http://www.ros.org/doc/api/sensor_msgs/html/namespacesensor__msgs_1_1image__encodings.html for more info.
#include <sensor_msgs/image_encodings.h>
//Include headers for OpenCV Image processing
#include <opencv2/imgproc/imgproc.hpp>
//Include headers for OpenCV GUI handling
#include <opencv2/highgui/highgui.hpp>
#include <string>
#include <cstdio>
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <cmath>


using namespace cv;
using namespace std;

//global variables
Mat src, dst;
Scalar m, st;
int width = 120;
int height = 120;
int thresh = 10;
int h, w, h1, w1;

//Store all constants for image encodings in the enc namespace to be used later.
namespace enc = sensor_msgs::image_encodings;

//Declare a string with the name of the window that we will create using OpenCV where processed images will be displayed.
static const char WINDOW[] = "adaptive";
static const char TRKBAR[] = "Trackbar_Window";

//Use method of ImageTransport to create image publisher
image_transport::Publisher pub;

void on_trackbar(int , void*){
    dst = Mat::zeros(src.size(),CV_8UC1);

    h = src.rows;
    w = src.cols;

    for (int k = 0; k < (w/width); ++k)
    {   
        for (int l = 0; l < (h/height); ++l)
        {
            Rect r (k*width, l*height, width, height);
            Mat smallImg = src(r).clone();
            cv::cvtColor(smallImg, smallImg, CV_BGR2GRAY);
            cv ::meanStdDev(smallImg, m, st);

            h1 = smallImg.rows;
            w1 = smallImg.cols;

            for (int i = 0; i < h1; ++i)
            {
                for (int j = 0; j < w1; ++j)
                {
                //std::cout<<j<<endl;
                    double px = smallImg.at<uchar>(i,j);

                    if ((px - m[0]) > thresh)
                    smallImg.at<uchar>(i,j) =saturate_cast<uchar>( (px - m[0])/st[0])*100;
                    else
                    smallImg.at<uchar>(i, j) = 0;               
                }
            }

            smallImg.copyTo(dst(r));
        }

    }
}

//This function is called everytime a new image is published
void imageCallback(const sensor_msgs::ImageConstPtr& original_image){
	//Convert from the ROS image message to a CvImage suitable for working with OpenCV for processing
    createTrackbar("width", TRKBAR, &width, 1000, on_trackbar);
    createTrackbar("height", TRKBAR, &height, 1000, on_trackbar);
    createTrackbar("thresh", TRKBAR, &thresh, 100, on_trackbar);

	cv_bridge::CvImagePtr cv_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
        //OpenCV expects color images to use BGR channel order.
        cv_ptr = cv_bridge::toCvCopy(original_image , enc::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("videofeed::main.cpp::cv_bridge exception: %s", e.what());
		 return;
	};
	src = cv_ptr->image;
	on_trackbar(0, 0);
    //Add some delay in miliseconds. The function only works if there is at least one HighGUI window 
	//created and the window is active. If there are several HighGUI windows, any of them can be active.
    cv::waitKey(3);
    imshow("source_window",src);
    imshow("dst",dst);
    /**
    * The publish() function is how you send messages. The parameter
    * is the message object. The type of this object must agree with the type
    * given as a template parameter to the advertise<>() call, as was done
    * in the constructor in main().
    */
    //Convert the CvImage to a ROS image message and publish it on the "camera/image_processed" topic.
        pub.publish(cv_ptr->toImageMsg());
}
/**
* This tutorial demonstrates simple image conversion between ROS image message and OpenCV formats and image processing
*/
int main(int argc, char **argv)
{
	/**
    * The ros::init() function needs to see argc and argv so that it can perform
    * any ROS arguments and name remapping that were provided at the command line. For programmatic
    * remappings you can use a different version of init() which takes remappings
    * directly, but for most command-line programs, passing argc and argv is the easiest
    * way to do it.  The third argument to init() is the name of the node. Node names must be unique in a running system.
    * The name used here must be a base name, ie. it cannot have a / in it.
    * You must call one of the versions of ros::init() before using any other
    * part of the ROS system.
    */
    ros::init(argc, argv, "image_processor");
    /**
    * NodeHandle is the main access point to communications with the ROS system.
    * The first NodeHandle constructed will fully initialize this node, and the last
    * NodeHandle destructed will close down the node.
    */
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
    //OpenCV HighGUI call to create a display window on start-up.
    namedWindow("source_window", CV_WINDOW_AUTOSIZE );
    //namedWindow("gray scale", CV_WINDOW_AUTOSIZE );
    namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
    namedWindow(TRKBAR, CV_WINDOW_AUTOSIZE);
    /**
    * Subscribe to the "camera/image_raw" base topic. The actual ROS topic subscribed to depends on which transport is used.
    * In the default case, "raw" transport, the topic is in fact "camera/image_raw" with type sensor_msgs/Image. ROS will call
    * the "imageCallback" function whenever a new image arrives. The 2nd argument is the queue size.
    * subscribe() returns an image_transport::Subscriber object, that you must hold on to until you want to unsubscribe.
    * When the Subscriber object is destructed, it will automatically unsubscribe from the "camera/image_raw" base topic.
    */
    image_transport::Subscriber sub = it.subscribe("/camera/image_raw",1,imageCallback);
    //OpenCV HighGUI call to destroy a display window on shut-down.
    //cv::destroyWindow(WINDOW);
    /**
    * The advertise() function is how you tell ROS that you want to
    * publish on a given topic name. This invokes a call to the ROS
    * master node, which keeps a registry of who is publishing and who
    * is subscribing. After this advertise() call is made, the master
    * node will notify anyone who is trying to subscribe to this topic name,
    * and they will in turn negotiate a peer-to-peer connection with this
    * node.  advertise() returns a Publisher object which allows you to
    * publish messages on that topic through a call to publish().  Once
    * all copies of the returned Publisher object are destroyed, the topic
    * will be automatically unadvertised.
    *
    * The second parameter to advertise() is the size of the message queue
    * used for publishing messages.  If messages are published more quickly
    * than we can send them, the number here specifies how many messages to
    * buffer up before throwing some away.
    */
    pub = it.advertise("/camera/output",1);
    /**
    * In this application all user callbacks will be called from within the ros::spin() call.
    * ros::spin() will not return until the node has been shutdown, either through a call
    * to ros::shutdown() or a Ctrl-C.
    */
    ros::spin();
    //ROS_INFO is the replacement for printf/cout.
    ROS_INFO("videofeed::main.cpp::No error.");
}