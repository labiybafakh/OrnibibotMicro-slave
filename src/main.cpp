#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Thread.h>
#include <ThreadController.h>
#include "AS5048A.h"
#include <vector>
#include <string>

#include "OrnibibBot.hpp"

#include <iostream>
#include <iomanip>
#include <ros.h>
#include <sensor_msgs/JointState.h>
#include <ornibibot/joint_data.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

OrnibiBot robot;

AS5048A angle_sensor(SS, true);

char *joint_name[2]={"wing_left", "wing_right"};
float joint_position[2];
float previous_joint_position[2];
float joint_velocity[2];
float joint_threshold[2];

struct wing_position{
  float left;
  float right;
};

struct wing_velocity{
  float left;
  float right;
};


float threshold_left = 1.57;
float threshold_right = 0.77;


bool flapMode;

const char* ssid     = "ntlab1802";
const char* password = "hoge1802";
// Set the rosserial socket server IP address
IPAddress server(192,168,30,243);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
ornibibot::joint_data wing_left;
ornibibot::joint_data wing_right;

volatile double freqFlap=4;
volatile double pTail;
volatile double rTail;
volatile uint16_t counter=0;
bool failSafe;
bool lostFrame;

int _Servo[5];
int* _targetServo = _Servo ;


ros::Publisher node_wing_left("wing_left", &wing_left);
ros::Publisher node_wing_right("wing_right", &wing_right);

ros::Time current_time, previous_time;
float dt;
float *p_dt = &dt;

struct wing_position now_wing_pos, prev_wing_pos;
struct wing_velocity _wing_velocity;

struct wing_position *p_wing_position = &now_wing_pos;
struct wing_velocity *p_wing_velocity = &_wing_velocity;



void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    // if (nh.connected()) {
      digitalWrite(2, HIGH);

      current_time = nh.now();

      //Wing Left
      // wing_left.header.stamp = current_time; 
      // wing_left.name.data = "Wing Left";
      // wing_left.position.data = p_wing_position->left;
      // wing_left.velocity.data = p_wing_velocity->left;

      // node_wing_left.publish(&wing_left);

      //Wing Right
      wing_right.header.stamp = current_time; 
      wing_right.name.data = "Wing Right";
      wing_right.position.data = p_wing_position->right;
      wing_right.velocity.data = p_wing_velocity->right;

      node_wing_right.publish(&wing_right);

    // }
  
    nh.spinOnce();
    delay(5);
  } 
}

void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
      // now_wing_pos.left = -(angle_sensor.getRotationInRadians() - threshold_left);
      now_wing_pos.right  = angle_sensor.getRotationInRadians() - threshold_right;

      // _wing_velocity.left =  (now_wing_pos.left-prev_wing_pos.left)/0.01;
      _wing_velocity.right =  (now_wing_pos.right-prev_wing_pos.right)/0.01;

      // prev_wing_pos.left = now_wing_pos.left;
      prev_wing_pos.right = now_wing_pos.right;

      delay(5);
      
  }
}

void setup()
{
  pinMode(2, OUTPUT);
  Serial.begin(115200);
  angle_sensor.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();
  // nh.setSpinTimeout(15);
  // nh.advertise(node_wing_left);
  nh.advertise(node_wing_right);
  
  xTaskCreatePinnedToCore(
                    paramUpdate,   /* Task function. */
                    "Task1",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task1,      /* Task handle to keep track of created task */
                    0);          /* pin task to core 0 */
  delay(500);
  xTaskCreatePinnedToCore(
                    motorUpdate,   /* Task function. */
                    "Task2",     /* name of task. */
                    10000,       /* Stack size of task */
                    NULL,        /* parameter of the task */
                    1,           /* priority of the task */
                    &Task2,      /* Task handle to keep track of created task */
                    1);          /* pin task to core 0 */
  delay(500);

  // nh.spinOnce();
}

void loop()
{ 
 
}