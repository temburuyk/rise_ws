#include "ros/ros.h"

#include "roboteq/RoboteqDevice.h"
#include "roboteq/roboteq_msg.h"
#include "roboteq/ErrorCodes.h"
#include "roboteq/Constants.h"

#include "math.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"

using namespace std;

float desired_heading = 90;
float max_vel = 0;
float max_ang_vel = 180;
float desired_velocity=0;
int pulse3 = 0;
int Din4=0;
bool e_stop = true;
float last_head = 0;
int il = 0;


const float RAD_TO_DEG = 180/3.141592;
const float DEG_TO_RAD = 3.141592/180;

const float Lr = 0.54;       
const float CIRC = 0.942; 
const float RED = 8.3;

const float correction_factor_ang = .94714286;
const float correction_factor_vel = 1.0;

const int turn_direction = 1; 
const int power_direction = -1; 

//roboteq::roboteq_msg output;
float w, v;

int status=0;
RoboteqDevice device;

float filter = 0.1;
int ch1_rpm,ch2_rpm;
int ch1_rpm_old,ch2_rpm_old;
int ch1_encoder_value,ch2_encoder_value;
roboteq::roboteq_msg output;
 

const float P_O=0.75, P_WO=0.68, I=0.01, D=0.1;//float P=0.8, I=0.01, D=0.01;
float P=0.6;
float err_new=1, err_old, errd, erri;

bool velocity_received(false), desired_heading_received(false); 

float compute_error(float yaw){
	yaw -= desired_heading;
	if(yaw>180) yaw -= 360;
	else if (yaw<-180) yaw += 360;

	return yaw;
}

void motorcommand(float errd_1, float erri_1){
	float pid = (P*err_new + I*erri_1 + D*errd_1);
	if (pid < (-max_ang_vel)) pid = -max_ang_vel;
	if (pid > max_ang_vel) pid = max_ang_vel;

	int _max = 1300; 
	device.GetConfig(_MXRPM, _max); 
	float max = _max;

	w = pid*correction_factor_ang*DEG_TO_RAD*(Lr/2)*(60/CIRC)*(float)turn_direction*(1000/max)*RED; 
			
	v = desired_velocity*correction_factor_vel*(60/CIRC)*(float)power_direction*(1000/max)*RED;                    		
	//cout<<v<<" "<<w<<endl;
	cout<<"Desired Vel:"<<desired_velocity<<" & "<<v<<"  ||  "<<"Desired Pid:"<<pid<<" "<<" & "<<w<<endl;

	if( (fabs(v+w) > 2000) || (fabs(v-w) > 2000) ){ 
		device.SetCommand(_G, 1, 0); device.SetCommand(_G, 2, 0);
	}
	else if (velocity_received == true && desired_heading_received == true && e_stop==false){
			status = device.SetCommand(_G, 1, int(w));	
			status = device.SetCommand(_G, 2, int(v));
			if(il == 10) device.SetCommand(_DSET, 2);
			if(il == 20) device.SetCommand(_DRES, 2);
	}	
	else if (velocity_received == true && desired_heading_received == true && e_stop==true){
			status = device.SetCommand(_G, 1, 0);	
			status = device.SetCommand(_G, 2, 0);
			device.SetCommand(_DSET, 2);
			cout<<"Emergency_STOP"<<endl;
	}		
	
	velocity_received = false;
}

void PID(const geometry_msgs::Pose2D::ConstPtr& msg){
	// std::cout<<"YO1"<<std::endl;	
//	float yaw = info->linear.theta;
	float yaw = msg->theta;

	float dt = 0.05;
	
	err_old = err_new; 
	err_new = compute_error(yaw);
		
	errd = (err_new-err_old)/dt;
	if (errd < (-20)) errd = -20;
	if (errd > 20) errd = 20;

	erri += (err_new + err_old)*dt/2;	
	if (erri < (-20)) erri = -20;
	if (erri > 20) erri = 20;
	
	motorcommand(errd, erri);
}

void deshead(const std_msgs::Float32::ConstPtr& msg_in){
	desired_heading = msg_in->data;
	if(desired_heading )
	last_head = desired_heading;
	desired_heading_received = true;
}

void desvel(const std_msgs::Float32::ConstPtr& msg_in1){
	desired_velocity = msg_in1->data;
	velocity_received = true;
}

void set_p(const std_msgs::Bool::ConstPtr& msg_in){
	if(msg_in->data==true) P=P_O;
	else P=P_WO;
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
	
	ros::Subscriber heading             = n.subscribe("/imu/HeadingTrue_degree", 1, PID);
	ros::Subscriber desired_heading     = n.subscribe("/desired_bearing", 1, deshead);
	ros::Subscriber desired_velocity    = n.subscribe("/desired_velocity", 1, desvel);
	ros::Subscriber pid_type            = n.subscribe("/st_obj_stat", 1, set_p);
	ros::Publisher feed_rpm             = n.advertise<roboteq::roboteq_msg>("/roboteq_monitor", 1);
	//ros::Publisher  feedback            = n.advertise<roboteq::roboteq_msg>("/roboteq_monitor", 1);


	while(ros::ok()){
		device.GetValue(_PLSIN, 3, pulse3);
		//device.GetValue(_DIN,   4, Din4);
		il = il +1;
		if(il>20){il = 0;}
		if(pulse3>1600){e_stop = false;}
		else{e_stop=true;}
		//e_stop = false;
		/* ch1_rpm_old = ch1_rpm;
        ch2_rpm_old = ch2_rpm;
		device.GetValue(_S, 1, ch1_rpm);
		device.GetValue(_S, 2, ch2_rpm);
		device.GetValue(_C, 1, ch1_encoder_value);
		device.GetValue(_C, 2, ch2_encoder_value);
		ch1_rpm= (1-filter)*ch1_rpm+filter*ch1_rpm_old;
		ch2_rpm= (1-filter)*ch2_rpm+filter*ch2_rpm_old;

    	output.header.stamp = ros::Time::now();
		//RPM Stuff
		output.rpm_1 = ch1_rpm; output.rpm_2 = ch2_rpm;
		//Encoder Stuff
		output.encoder_1 = ch1_encoder_value; output.encoder_2 = ch2_encoder_value;

		feed_rpm.publish(output);*/
		ros::spinOnce();
		loop_rate.sleep();
	}

	device.SetCommand(_DSET, 2);
	
	return 0;
}
