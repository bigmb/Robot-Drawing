/*
 * rosserial PubSub Example
 * Prints "hello world!" and toggles led
 */

#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <RedBot.h>
#include <Servo.h>

ros::NodeHandle  nh;
RedBotMotors motors;
RedBotEncoder encoder = RedBotEncoder(A2, 10);  // initializes encoder on pins A2 and 10
int buttonPin = 12;
int countsPerRev = 192;
//Servo servo_left, servo_right;

void motor_left_sub( const std_msgs::Int16& msg){
  motors.leftMotor(msg.data);
  //servo_left.write(msg.data);
}

void motor_right_sub( const std_msgs::Int16& msg){
  motors.rightMotor(msg.data);
  //servo_right.write(msg.data);
}

ros::Subscriber<std_msgs::Int16> sub_left("/motor_left", motor_left_sub );
ros::Subscriber<std_msgs::Int16> sub_right("/motor_right", motor_right_sub );


std_msgs::Int16 encode_left, encode_right;
//std_msgs::String str_msg;
ros::Publisher left_encoder("encoder_left", &encode_left);
ros::Publisher right_encoder("encoder_right", &encode_right);

//char hello[13] = "hello world!";



void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  
  //pinMode(13, OUTPUT);
  nh.initNode();
  nh.advertise(left_encoder);
  nh.advertise(right_encoder);
  nh.subscribe(sub_left);
  nh.subscribe(sub_right);
  
}

void loop()
{
  //str_msg.data = hello;
  // store the encoder counts to a variable.
  encode_left.data = -encoder.getTicks(LEFT);    // read the left motor encoder
  encode_right.data = encoder.getTicks(RIGHT);   // read the right motor encoder
  
  left_encoder.publish( &encode_left);
  right_encoder.publish( &encode_right);
  nh.spinOnce();
  delay(500);
}
