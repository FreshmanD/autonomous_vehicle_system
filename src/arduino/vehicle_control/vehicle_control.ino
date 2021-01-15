#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>

ros::NodeHandle  nh;
geometry_msgs::Twist msg;

const int PWMpin1=5; //FTM1 timer Thrust ESC
const int PWMpin2=20; //FTM0 timer Sterring Servo

boolean back_lock = true;

float thrustNeutral = 39;
float thrustForwardsMax = 43;
float thrustBackwardsMax = thrustNeutral - (thrustForwardsMax - thrustNeutral);

float angleNeutral = 39;
float angleLeftMax = 29;
float angleRightMax = angleNeutral + (angleNeutral - angleLeftMax);

float inThrustMin = -1;  
float inThrustMax =  1;
float inAngleMin =  0.3;
float inAngleMax =  -0.3;


void callback( const geometry_msgs::Twist& cmd_vel){
  float thrust = cmd_vel.linear.x;
  float angle = cmd_vel.angular.z;
  
  setSpeed(thrust);
  setAngle(angle);
  
}

ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", callback);

void setup() {
  pinMode(PWMpin1,OUTPUT); 
  analogWriteFrequency(PWMpin1, 100);
  pinMode(PWMpin2,OUTPUT);
  arm();

  nh.initNode();
  nh.subscribe(sub);
  
}

void arm()
{
//arm speed controller, modify as necessary for your ESC

  Serial.println("Arming");
  setSpeed(39);
  delay(1000);
  setAngle(39);
  delay(1000);
  Serial.println("Armed");
}

float fmap (float toMap, float in_min, float in_max, float out_min, float out_max) {
  return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setSpeed(float thrust)
{
  float finalThrust = fmap(thrust, inThrustMin, inThrustMax, thrustBackwardsMax, thrustForwardsMax);
  analogWrite(PWMpin1,finalThrust);
}

void setAngle(float angle)
{
  float finalAngle = fmap(angle, inAngleMin, inAngleMax, angleLeftMax, angleRightMax);
  analogWrite(PWMpin2,finalAngle);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
