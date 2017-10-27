#include "ros/ros.h"

#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"

#include "math.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Point.h"
#define PI 3.14159265
float b = 510;
float l = 810;
float w = 625;
float velocity;
float v_l, v_r;
float s_a[2];
int status=0;
int pulse3 = 0;
RoboteqDevice device;
float filter = 0.1;
int ch1_rpm,ch2_rpm;
int ch1_rpm_old,ch2_rpm_old;
int ch1_encoder_value,ch2_encoder_value;
roboteq::roboteq_msg output;
void edf_function(const geometry_msgs::Point::ConstPtr& steering_angle)
{	
	s_a[0] = steering_angle->x;
	s_a[1] = steering_angle->y;
	float delta_r, delta_l;
	float delta_i, delta_o;
	delta_r = s_a[1] / 0.875;
	delta_l = delta_r * 0.75;
	if(s_a[1] = 0){
		v_l = s_a[0];
		v_r = s_a[0];
	}
	else if(s_a[1] > 0){
		delta_i = delta_r;
		delta_o = delta_l;
		float r_i, r_o, omega, r_c;
		r_i = l/tan(delta_i*PI/180) - (w-b)/2;
		r_o = l/tan(delta_o*PI/180) + (w-b)/2;
		r_c = l/tan(delta_i*PI/180) + b/2;
		omega = s_a[0]/r_c;
		v_r = r_i * omega;
		v_l = r_o * omega;
	}
	else{
		delta_i = -1*delta_l;
		delta_o = -1*delta_r;
		float r_i, r_o, omega, r_c;
		r_i = l/tan(delta_i*PI/180) - (w-b)/2;
		r_o = l/tan(delta_o*PI/180) + (w-b)/2;
		r_c = l/tan(delta_i*PI/180) + b/2;
		omega = s_a[0]/r_c;
		v_l = r_i * omega;
		v_r = r_o * omega;
	}
	status = device.SetCommand(_G, 1, int(v_r)*400);	
	status = device.SetCommand(_G, 2, int(v_l)*400);

}
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

ros::Subscriber edf_sub  = n.subscribe("/sa_vel", 1, edf_function);

	while(ros::ok()){
		device.GetValue(_PLSIN, 3, pulse3);

			ros::spinOnce();
		loop_rate.sleep();
	}

	device.SetCommand(_DSET, 2);
	
	return 0;
}
