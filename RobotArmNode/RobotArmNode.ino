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

#include <ros.h>
#include <robot_comm_msg/AngleArr.h>
#include <Adafruit_PWMServoDriver.h> // for servo motor
#include <std_msgs/String.h>
#include <Stepper.h> // for stepper motor

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Pins we read angles from
const int analog1 = 6;
const int analog2 = 7;

// Current step
int cStep = 0;
int targetStep = 0;

// STEPPER WIRING
// Pin 8 to IN1 on the ULN2003 driver
// Pin 9 to IN2 on the ULN2003 driver
// Pin 10 to IN3 on the ULN2003 driver
// Pin 11 to IN4 on the ULN2003 driver

const int stepsPerRevolution = 2048;
Stepper myStepper = Stepper(stepsPerRevolution, 8, 10, 9, 11);

ros::NodeHandle  nh;

std_msgs::String str_msg;
ros::Publisher arm_status("arm_status", &str_msg);
char status[10] = "arm moved";


void servo_cb( const robot_comm_msg::AngleArr& cmd_msg){
  int angle0 = max(0, min(cmd_msg.angles[0], 180));
  int angle1 = max(0, min(cmd_msg.angles[1], 180));
  int angle2 = max(0, min(cmd_msg.angles[2], 180));

  int targetSteps = ((float)angle0)/360.0 * stepsPerRevolution;
  int diff = cStep - targetSteps;
  cStep = targetSteps;
  myStepper.step(diff);
  
  int frq1 = map(angle1, 0, 180, 140, 500);
  pwm.setPWM(0, 0, frq1);
  int frq2 = map(angle2, 0, 180, 140, 500);
  pwm.setPWM(1, 0, frq2);

  digitalWrite(13, HIGH-digitalRead(13));  //toggle led 

  str_msg.data = status;
  arm_status.publish(&str_msg); 
}


ros::Subscriber<robot_comm_msg::AngleArr> sub("cmd_angle", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(arm_status);

  myStepper.setSpeed(5);
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
}

void loop(){
  nh.spinOnce();
  delay(1);
}
