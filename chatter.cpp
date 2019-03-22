/*
 * rosserial Subscriber Example
 * Blinks an LED on callback
 */

#include <ros.h>
#include <std_msgs/Empty.h>

#include <Arduino.h>
#include <maniros/MotorControl.h>

ros::NodeHandle nh;

const double EN=9;
const double M1D1=8; // vermutlich der PWM pin um die geschwindigkeit zu steuern
const double M1IN1=7;
const double M1IN2=6;
const double M1D2=5; //


void messageCb( const std_msgs::Empty& motor_msg){
  nh.loginfo("Arduino callack fired")
  digitalWrite(13, HIGH-digitalRead(13));
  
  //front_right_speed = (int) (255*motor_msg.data.front_right_speed);

  if (motor_msg.data.front_right_speed > 0) {
    digitalWrite(EN, HIGH);
    digitalWrite(M1D1, LOW); 
    digitalWrite(M1IN2, HIGH);
    digitalWrite(M1IN1, LOW);
    //digitalWrite(M1D2, HIGH); 
    analogWrite(M1D2, 255*motor_msg.data.front_right_speed);//PWM signal für die Ansteuerung der Geschwindigkeit
  }
  if (motor_msg.data.front_right_speed < 0) {
    digitalWrite(EN, HIGH);
    digitalWrite(M1IN2, LOW);
    digitalWrite(M1IN1, HIGH);
    analogWrite(M1D2, -255*motor_msg.data.front_right_speed);//PWM signal für die Ansteuerung der Geschwindigkeit
  }
  if (motor_msg.data.front_right_speed == 0) {
    digitalWrite(EN, LOW);
    digitalWrite(M1IN2, LOW);
    digitalWrite(M1IN1, LOW);
  }

}

ros::Subscriber<std_msgs::Empty> sub("motor_control", &messageCb );

void setup()
{
  pinMode(13, OUTPUT);

  pinMode(EN, OUTPUT);
  pinMode(M1D1, OUTPUT);
  pinMode(M1IN1, OUTPUT);
  pinMode(M1IN2, OUTPUT);
  pinMode(M1D2, OUTPUT);
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  digitalWrite(EN, LOW);
  nh.spinOnce();
  delay(1);
}
