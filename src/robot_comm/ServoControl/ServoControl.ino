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

#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh; //node handle for serial communication

Servo servo1;
Servo servo2;
Servo servo3; //a stepper motor in reality

void servo_cb( const std_msgs::UInt16& cmd_msg){

	
  servo1.write(cmd_msg.data); //set servo angle, should be from 0-180 
  servo2.write();
  servo3.write();
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}


ros::Subscriber<std_msgs::UInt16> sub("cmd_angle", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo1.attach(9); //attach it to pin 9
  servo2.attach(10); 
  servo3.attach(11); 
}

void loop(){
  nh.spinOnce();
  delay(1);
}
