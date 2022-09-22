#include <Arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <Thread.h>
#include <ThreadController.h>
#include "AS5048A.h"

#include "OrnibibBot.hpp"

#include <iostream>
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string.h>

TaskHandle_t Task1;
TaskHandle_t Task2;

OrnibiBot robot;

AS5048A angle_sensor(SS, true);

struct wing_position{
  int16_t left;
  int16_t right;
} _wing_position;

struct wing_position *p_wing_position = &_wing_position;

static uint8_t threshold_left = 90;
static uint8_t threshold_right = 44;


bool flapMode;

const char* ssid     = "ntlab1802";
const char* password = "hoge1802";
// Set the rosserial socket server IP address
IPAddress server(192,168,30,243);
// Set the rosserial socket server port
const uint16_t serverPort = 11411;

ros::NodeHandle nh;
// Make a chatter publisher
std_msgs::Int16 wing_left;
// std_msgs::Int16 wing_right;

volatile double freqFlap=4;
volatile double pTail;
volatile double rTail;
volatile uint16_t counter=0;
bool failSafe;
bool lostFrame;

int _Servo[5];
int* _targetServo = _Servo ;


ros::Publisher node_wing_left("wing_left", &wing_left);
// ros::Publisher node_wing_right("wing_right", &wing_right);


void paramUpdate( void * pvParameters ){
  Serial.print("Task1 running on core ");
  Serial.println(xPortGetCoreID());
  for(;;){
    if (nh.connected()) {
      digitalWrite(2, HIGH);

      wing_left.data = p_wing_position->left;
      node_wing_left.publish(&wing_left);

      // wing_right.data = p_wing_position->right;
      // node_wing_right.publish(&wing_right);

    }
  
    else {
      
      
    }
  nh.spinOnce();
  delay(1);
  } 
}

void motorUpdate( void * pvParameters ){
  Serial.print("Task2 running on core ");
  Serial.println(xPortGetCoreID());

  for(;;){
      _wing_position.left = -(angle_sensor.getRotationInDegrees() - threshold_left);
      // _wing_position.right = angle_sensor.getRotationInDegrees() - threshold_right;
      delay(1);
      
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

  // Set the connection to rosserial socket server
  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  // // Another way to get IP
  // Serial.print("IP = ");
  // Serial.println(nh.getHardware()->getLocalIP());

  // // Start to be polite
  nh.advertise(node_wing_left);
  // nh.advertise(node_wing_right);

  
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


}

void loop()
{ 
 
}