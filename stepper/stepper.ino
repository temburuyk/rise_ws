/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif
#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
int current_position=0;
int msg;

void stepper_cb( const std_msgs::Int16& cmd_msg){
  msg = cmd_msg.data; 
  int diff = msg-current_position;
  Serial.println(diff);
  if(diff==0)
  {}  
  else if(diff>0){
    digitalWrite(4,HIGH);
  for(int i=0;i<10;i++){
    digitalWrite(3,HIGH);
    delayMicroseconds(350);
    digitalWrite(3,LOW);
    delayMicroseconds(350);  }
    current_position = current_position+10;
    
} 
else{
  digitalWrite(4,LOW);
  for(int i=0;i<10;i++){
    digitalWrite(3,HIGH);
    delayMicroseconds(350);
    digitalWrite(3,LOW);
    delayMicroseconds(350);  }
    current_position=current_position-10;
    

}
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::Int16> sub("steering", stepper_cb);

void setup(){
  pinMode(13, OUTPUT);
  pinMode(3, OUTPUT);//step
  pinMode(4, OUTPUT);//dir
  pinMode(5, OUTPUT);//step-gnd
  pinMode(6, OUTPUT);//dir-gnd
  nh.initNode();
  nh.subscribe(sub);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
