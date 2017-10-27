#include "ros/ros.h"

#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"

using namespace std;
int status=0;
RoboteqDevice device;

float filter = 0.1;
int ch1_rpm,ch2_rpm;
int ch1_rpm_old,ch2_rpm_old;
int ch1_encoder_value,ch2_encoder_value;
roboteq::roboteq_msg output;


int main(int argc, char* argv[]){	
	
	ros::init(argc, argv, "trajec_design");
	ros::NodeHandle n;
	ros::Rate loop_rate(10);
		
	ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
	ROS_INFO("Initializing...");
	
	status = device.Connect("/dev/ttyACM0");
	if (status != 0){
		device.Connect("/dev/ttyACM1");
	}
	ch1_rpm = 0;
	ch2_rpm = 0;

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d\n", status);
		ROS_INFO("Attempting server restart...");
		usleep(1000000);
		device.Disconnect();

		status = device.Connect("/dev/ttyACM0");
		if (status != 0){
			device.Connect("/dev/ttyACM1");
		}
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
	
	int mode=0;
	device.GetConfig(_MXMD, mode);
	ROS_INFO("Operating mode: %d", mode);
	ros::Duration(1.0).sleep();
	
	
	device.SetConfig(_MXRPM, 1, 1300);
	device.SetConfig(_MXRPM, 2, 1300);