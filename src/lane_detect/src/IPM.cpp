#include "image_properties.h"
#include "nav_msgs/OccupancyGrid.h"
//#include "videofeed/lane.h"
//#include "videofeed/multi_lane.h"
//#include "videofeed/calib.h"
//#include "videofeed/multi_calib.h"
#include "math.h"

using namespace cv;
using namespace std;

namespace enc = sensor_msgs::image_encodings;
string WINDOW = "IPM";

nav_msgs::OccupancyGrid Final_Grid;

//videofeed::multi_calib outdata;
//videofeed::calib Lane_Data;
ros::Subscriber sub_Lanedata_left;
ros::Subscriber sub_Lanedata_center;
ros::Subscriber sub_Lanedata_right;

ros::Publisher pub_Lanedata;

std::vector<std::vector<Point> > Lane_points_left;
std::vector<std::vector<Point> > Lane_points_center;
std::vector<std::vector<Point> > Lane_points_right;

Mat src_left, src_center, src_right;
Mat roi, final_grid;

//final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC3);

float topleftdist = 392;
float toprightdist = 374;
float bottomleftdist = 160;
float bottomrightdist = 146;

float topside = 357;
float bottomside = 187;
float nearpoint = 137;
float height = 189;
bool middledata;



void middleimage(const sensor_msgs::ImageConstPtr& original_image)
{
	Mat temp = Mat::zeros(Size(image_height, image_width), CV_8UC1);
	src_center = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
    
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(original_image, enc::MONO8);
	}
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("videofeed::igvc_IPM.cpp::cv_bridge exception: %s", e.what());
        return;
    }

	cv::Mat src = cv_ptr->image;
	// cout<<"Middle"<<endl;
	// cout<<src.rows<<" "<<src.cols<<endl;

	// Point2f p[4] = {Point(0, 230), Point(0, image_height - 1), Point(image_width - 1, 230), Point(image_width - 1, image_height - 1)};
	// Point2f q[4] = {Point(65, 326), Point(84, 370), Point(137, 333), Point(119, 371)};

	Point2f p[4] = {Point(0, 0), Point(639,0), Point(639, 479), Point(0, 479)};
	Point2f q[4] = {Point(153, 308), Point(346, 308), Point(273, 394), Point(226, 393)};

	Mat transform (3, 3, CV_32FC1);
	transform = getPerspectiveTransform(p, q);

	//roi = src_center(Rect(q[0].x - 1, q[0].y - 1, 53, 45));

	warpPerspective(src, roi, transform, Size(occ_grid_width, occ_grid_height));
	//imshow("roi",roi);
	for (int i = 0; i < Lane_points_center.size(); ++i)
	{
		Lane_points_center[i].clear();
	}

	Mat MiddleROI;
	roi.copyTo(MiddleROI);
	//imshow("MiddleROI", MiddleROI);

	if (!final_grid.data)
	final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
	// cout<<MiddleROI.rows<<" "<<MiddleROI.cols<<endl;
	// cout<<roi.rows<<" "<<roi.cols<<endl;

	final_grid = final_grid + MiddleROI;
	//final_grid.at<uchar>(100,100)=255;
	/*for (int k = occ_grid_height; k > 0; --k)
	{
		for (int i = 0; i < occ_grid_width; ++i)
		{
			if(final_grid.at<uchar>(i,k) > 1)
				{
					
					for (int j = k; j < occ_grid_height; ++j)
					{
						final_grid.at<uchar>(i,j) = final_grid.at<uchar>(i,k);
						//cout<<"here";
						
					}
				}
		}
	}*/

	Lane_points_center.clear();

	// cout<<src_center.rows<<" "<<src_center.cols<<endl;
	// cout<<roi.rows<<" "<<roi.cols<<endl;
	// cout<<temp.rows<<" "<<temp.cols<<endl;

	//imshow("final_grid", final_grid);

	//imshow("roi", roi);

	//imshow("temp", temp);

	// imshow("final_grid", final_grid);

	waitKey(1);

	// final_grid.release();

	src_center.release();
}



int main(int argc, char **argv)
{
	ros::init(argc, argv, "Lane_Occupancy_Grid");

	ros::NodeHandle nh;
	image_transport::ImageTransport it(nh);

	image_transport::Publisher pub;
	
    
	//image_transport::Subscriber sub_left = it.subscribe("/camera/image_raw1", 1, leftimage);
	image_transport::Subscriber sub_middle = it.subscribe("/camera/image_raw2", 1, middleimage);
	//image_transport::Subscriber sub_right = it.subscribe("/camera/image_raw3", 1, rightimage);

	cv::namedWindow(WINDOW, CV_WINDOW_AUTOSIZE);
	//namedWindow("Leftoccgrid", CV_WINDOW_AUTOSIZE);
	//namedWindow("Centeroccgrid", CV_WINDOW_AUTOSIZE);
	//namedWindow("Rightoccgrid", CV_WINDOW_AUTOSIZE);

	//namedWindow("Left Cam Trackbar", CV_WINDOW_AUTOSIZE);
	//namedWindow("Center Cam Trackbar", CV_WINDOW_AUTOSIZE);
	//namedWindow("Right Cam Trackbar", CV_WINDOW_AUTOSIZE);
	//namedWindow("Interpolater-Trackbar", CV_WINDOW_AUTOSIZE);

	pub = it.advertise("/Interpolater", 1);


	// sub_Lanedata_left = nh.subscribe("/caliberation1", 1, leftfusion);
	// sub_Lanedata_center = nh.subscribe("/caliberation2", 1, centerfusion);
	// sub_Lanedata_right = nh.subscribe("/caliberation3", 1, rightfusion);

	//pub_Lanedata = nh.advertise<nav_msgs::OccupancyGrid>("/Lane_Occupancy_Grid", 1);
	// sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();

	ros::Rate loop_rate(15);

	while(ros::ok())
	{

		final_grid = Mat::zeros(Size(occ_grid_width, occ_grid_height), CV_8UC1);
		ros::spinOnce();
		final_grid = final_grid(Rect(150,0,200,400));
		cvtColor(final_grid,final_grid,CV_GRAY2BGR);
		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", final_grid).toImageMsg();
		pub.publish(msg);
		/*Final_Grid.info.map_load_time = ros::Time::now();
		Final_Grid.header.frame_id = "lane";
	    Final_Grid.info.resolution = (float)map_width/(100*(float)occ_grid_widthr);
	    Final_Grid.info.width = 200;
	    Final_Grid.info.height = 400;

	    Final_Grid.info.origin.position.x = 0;
	    Final_Grid.info.origin.position.y = 0;
	    Final_Grid.info.origin.position.z = 0;

	    Final_Grid.info.origin.orientation.x = 0;
	    Final_Grid.info.origin.orientation.y = 0;
	    Final_Grid.info.origin.orientation.z = 0;
	    Final_Grid.info.origin.orientation.w = 1;

	    //cvtColor(final_grid,final_grid, CV_BGR2GRAY);

	    for (int i = 0; i < final_grid.rows; ++i)
	    {
	        for (int j = 0; j < final_grid.cols; ++j)
	        {
	        	if ( final_grid.at<uchar>(i,j) > 0)
	        	{
	        		//cout<<"r";
	        		Final_Grid.data.push_back(2);
	        	}
	        	else
	        		Final_Grid.data.push_back(final_grid.at<uchar>(i,j));
	        }
	    }   

    	pub_Lanedata.publish(Final_Grid);
    	waitKey(1);
		*/
		imshow(WINDOW, final_grid);
		//Final_Grid.data.clear();
		final_grid.release();
		loop_rate.sleep();
	}

	ROS_INFO("videofeed::occupancygrid.cpp::No error.");
}