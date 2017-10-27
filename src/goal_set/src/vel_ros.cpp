#include <bits/stdc++.h>
#include "ros/ros.h"
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
using namespace std;
double v_in=0; //initial velocity
double acc=1;
double decc=1;

ros::Publisher vel;
std_msgs::Float64MultiArray v;

std::vector<double> vmax(double c[], int n){
   std::vector<double> v_m;
	v_m.push_back(v_in);
   for (int i=0; i<n; i++)
      if(c[i]<=0.1623)
         v_m.push_back(5.55);
       else
         v_m.push_back(sqrt(0.5/c[i]));
		
	return v_m;
}

std::vector<std::vector<double> > segment(std::vector<double> v){
  int n=0;
  std::vector<std::vector<double> > x;
  while(n<v.size()){
    std::vector<double> row;
    while(n<v.size() && v[n]>=v[n+1]){
      row.push_back(v[n]);
      n++;}
    n++;
    x.push_back(row);}
  return x;
}

void smooth(std::vector<double> &v_m, std::vector<double> &distance){
	for(int i=1;i<v_m.size();i++){
		if((v_m[i-1]-v_m[i])>1){
			for(int j=i; j>=0; j--){
				double val2 = sqrt( v_m[i]*v_m[i]+2*acc*(distance[i]-distance[j]) ) ;	
//			cout<<"val="<<val2<<"		"<<"v["<<j<<"]="<<v_m[j]<<"			"<<distance[i]-distance[j]<<endl;
				v_m[j]=min(val2,v_m[j]);}
		}
	}
}

void path_vel(const nav_msgs::Path::ConstPtr& msg){

cout<<" not checked path"<<endl;
	if(!msg->poses.empty())
	{
		cout<<" checked path"<<endl;
	int no_data=msg->poses.size();
 // double x[no_data], y[no_data];
  double c[no_data-2];
	cout<<" checked path 4"<<endl;
 v.data.push_back(v_in); //initial condition
 cout<<" checked path 5"<<endl;
	std::vector<double> s;
  std::vector<double> v_m;
  std::vector<std::vector<double> > segment_v_m;


	
	for(int i=1; i<no_data-1; i++){
		//cout<<" checked path 2"<<endl;
	double a=sqrt(((msg->poses[i].pose.position.x-msg->poses[i-1].pose.position.x)*(msg->poses[i].pose.position.x-msg->poses[i-1].pose.position.x))+((msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y)*(msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y)));
	double b=sqrt(((msg->poses[i+1].pose.position.x-msg->poses[i].pose.position.x)*(msg->poses[i+1].pose.position.x-msg->poses[i].pose.position.x))+((msg->poses[i+1].pose.position.y-msg->poses[i].pose.position.y)*(msg->poses[i+1].pose.position.y-msg->poses[i].pose.position.y)));
	double e=sqrt(((msg->poses[i+1].pose.position.x-msg->poses[i-1].pose.position.x)*(msg->poses[i+1].pose.position.x-msg->poses[i-1].pose.position.x))+((msg->poses[i+1].pose.position.y-msg->poses[i-1].pose.position.y)*(msg->poses[i+1].pose.position.y-msg->poses[i-1].pose.position.y)));
	double d=(a+b+e)/3;
	int k = i-1;
	//cout<<" checked path 3"<<endl;
	//cout<<"a"<<(double)a<<"b"<<(double)b<<"e"<<(double)e<<endl;
	if(d==2*a/3 || d==2*b/3 || d==2*e/3)
		c[k]=0;
	else
		c[k]=(4*sqrt(d*(d-a)*(d-b)*(d-e))/(a*b*e));
	
	}
		cout<<"c"<<(double)(c[3])<<endl;
	s.push_back(0.0); 
  
	for(int i=1; i<no_data; i++)
    s.push_back(s[i-1]+sqrt(((msg->poses[i].pose.position.x-msg->poses[i-1].pose.position.x)*(msg->poses[i].pose.position.x-msg->poses[i-1].pose.position.x))+((msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y)*(msg->poses[i].pose.position.y-msg->poses[i-1].pose.position.y))));

 // for(int i=0;i<47;i++) cout<<s[i]<< " ";cout<<endl;
  v_m=vmax(c, no_data-2);
	
	//for(int i=0;i<v_m.size();i++) cout<<"vmax="<<v_m[i]<<" ";cout<<endl;
  segment_v_m=segment(v_m);
	smooth(v_m, s);

// for(int i=0;i<segment_v_m.size();i++){
//       for(int j=0;j<segment_v_m[i].size();j++) cout<<segment_v_m[i][j]<<" ";cout<<endl;}

  int a=0;
  int b=0;
  for(int i=0; i<segment_v_m.size();i++ ){
//cout<<"a"<<a<<" 		"<<b<<endl;
	for(int j=0; j<segment_v_m[i].size(); j++){
    	double val1 = sqrt(v.data[b]*v.data[b]+2*acc*(s[a]-s[b]));
      double val2 = sqrt( v_m[b+segment_v_m[i].size()-1]*v_m[b+segment_v_m[i].size()-1]+2*acc*(s[b+segment_v_m[i].size()-1]-s[a]) ) ;
      v.data[a] = min(v_m[a],val1);
      v.data[a] = min(v.data[a], val2);
      a++;}
//cout<<"val1="<<val1<<"      "<<"val2="<<val2<<"      "<<"vmax"<<v_m[a]<<'\n';
  b=b+segment_v_m[i].size();
	v.data[b]=v.data[b-1];
}
	
//	for(int i=0; i<47; i++){
//		std::cout<<v.data[i]<<'\n';}
	v_in=v.data[b-1];
	vel.publish(v);
	v.data.clear();
}
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "vel_ros");

  
  ros::NodeHandle n;

  
	ros::Subscriber path=n.subscribe<nav_msgs::Path>("/path",1,path_vel); 
 vel = n.advertise<std_msgs::Float64MultiArray>("/velocity", 1);

  ros::Rate loop_rate(200);

  
  int count = 0;
  while (ros::ok())
  {
    
    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }

  return 0;
}
