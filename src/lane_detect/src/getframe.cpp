#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <string>
#include <cmath>

using namespace std;
using namespace cv;

Mat src;
char str[50];

int main(int argc, char *argv[])
{
	int k = 1;
	int n;

	image_transport::Publisher pub;

    ros::init(argc, argv, "image_processor");
    ros::NodeHandle nh;
    //Create an ImageTransport instance, initializing it with our NodeHandle.
    image_transport::ImageTransport it(nh);
    pub = it.advertise("/camera/video",1);

	VideoCapture cap;
	
	if(argc > 1)
		cap.open(string(argv[1]));
	else
		cap.open(0);
	
	cv_bridge::CvImagePtr cv_ptr;

	namedWindow("Source_Image", CV_WINDOW_AUTOSIZE);
	rosbag play 
	while (waitKey(30) != 27)
	{
		// cout<<"Loop entered \n";
		// n = sprintf(str, "Frame_%d_4.jpg", k);
		// cout<<"sprintf() successful \n";
		// cout<<str;
		bool ok = cap.read(src);

		if(!ok || !src.data)
			break;
		imshow("Source_Image", src);

		if (ok && (src.rows > 0) && (src.cols > 0))
		cv_ptr->image = src;

        pub.publish(cv_ptr->toImageMsg());
		// imwrite(str, src);
		k++;
	}

	src.release();
	destroyAllWindows();
	return 0;
}