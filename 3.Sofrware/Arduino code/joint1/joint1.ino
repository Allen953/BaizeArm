#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <ESP32Servo.h>
#include <robot_msg/robotarm_7dof_jointstate.h>
//末端夹爪：80度为松开，110度为🏠夹住
Servo myservo1;  // create servo object to control a servo,joint4
Servo myservo2;  // create servo object to control a servo,joint5

#define servoPin1 12
#define servoPin2 13

#define pi 3.1416

#define LF_Motor1 32
#define LF_Motor2 33
#define LB_Motor1 26
#define LB_Motor2 25
#define RF_Motor1 17
#define RF_Motor2 5
#define RB_Motor1 18
#define RB_Motor2 19


#define Step_L1 LF_Motor1
#define Step_L2 LF_Motor2
#define Step_L3 LB_Motor1
#define Step_L4 LB_Motor2

#define Step_R1 RF_Motor1
#define Step_R2 RF_Motor2
#define Step_R3 RB_Motor1
#define Step_R4 RB_Motor2

#define StepVelo 1

void init_pin(){
  pinMode(LF_Motor1,OUTPUT);
  pinMode(LF_Motor2,OUTPUT);
  pinMode(LB_Motor1,OUTPUT);
  pinMode(LB_Motor2,OUTPUT);
  pinMode(RF_Motor1,OUTPUT);
  pinMode(RF_Motor2,OUTPUT);
  pinMode(RB_Motor1,OUTPUT);
  pinMode(RB_Motor2,OUTPUT);

  digitalWrite(LF_Motor1,LOW);
  digitalWrite(LF_Motor2,LOW);
  digitalWrite(LB_Motor1,LOW);
  digitalWrite(LB_Motor2,LOW);
  digitalWrite(RF_Motor1,LOW);
  digitalWrite(RF_Motor2,LOW);
  digitalWrite(RB_Motor1,LOW);
  digitalWrite(RB_Motor2,LOW);
}
//50个周期为1圈
void StepL_position(int Goal_pos){
  static long current_pos = 0;

  if(current_pos<Goal_pos)
  {
    while(current_pos<Goal_pos)
    {
      digitalWrite(Step_L1,HIGH);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,HIGH);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,HIGH);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,HIGH);
      delay(StepVelo);    
      current_pos++;
    }   
    Serial.print("current_pos:");
    Serial.print(current_pos);
    Serial.print("  ");
    Serial.print("Goal_pos:");
    Serial.println(Goal_pos);
     
  }
  else if(current_pos>Goal_pos)
  {
    while(current_pos>Goal_pos)
    {
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,HIGH);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,HIGH);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,LOW);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,HIGH);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);
      digitalWrite(Step_L1,HIGH);
      digitalWrite(Step_L2,LOW);
      digitalWrite(Step_L3,LOW);
      digitalWrite(Step_L4,LOW);
      delay(StepVelo);    
      current_pos--;
    }    
  }
  
  Serial.println(current_pos);
}

void initservo(){
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  myservo1.setPeriodHertz(50);    // standard 50 hz servo
  myservo1.attach(servoPin1, 500, 2500); // attaches the servo on pin 

  myservo2.setPeriodHertz(50);    // standard 50 hz servo
  myservo2.attach(servoPin2, 500, 2500); // attaches the servo on pin 
  
}

//
// WiFi Definitions //
//
const char* ssid = "S725";
const char* password = "s725s725";
 
IPAddress server(192, 168, 1, 111); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;
 
WiFiClient client;
 
class WiFiHardware {
 
  public:
  WiFiHardware() {};
 
  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11411);
  }
 
  // read a byte from the serial port. -1 = failure
  int read() {
    // implement this method so that it reads a byte from the TCP connection and returns it
    //  you may return -1 is there is an error; for example if the TCP connection is not open
    return client.read();         //will return -1 when it will works
  }
 
  // write data to the connection to ROS
  void write(uint8_t* data, int length) {
    // implement this so that it takes the arguments and writes or prints them to the TCP connection
    for(int i=0; i<length; i++)
      client.write(data[i]);
  }
 
  // returns milliseconds since start of program
  unsigned long time() {
     return millis(); // easy; did this one for you
  }
};
 
int i;
float servo1_pos = 90.0;
bool servo2_f = 0;//0为松开，1为夹住
 
void chatterCallback(const robot_msg::robotarm_7dof_jointstate& robotarm_joint) {
   
   servo1_pos = 180.0*robotarm_joint.position[3]/pi;
   Serial.println(servo1_pos);
   servo2_f = robotarm_joint.zhuazi;
   StepL_position(int(180.0*robotarm_joint.position[0]/pi));
}
 
 
std_msgs::String str_msg;
ros::Publisher chatter("esp_step", &str_msg);
ros::Subscriber<robot_msg::robotarm_7dof_jointstate> sub("robotarm_joint", &chatterCallback);
ros::NodeHandle_<WiFiHardware> nh;
char hello[30] = "ESP32 stepservo alive!";
 
 
void setupWiFi()
{
  
  WiFi.begin(ssid, password);
  Serial.print("\nConnecting to "); Serial.println(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) delay(500);
  if(i == 21){
    Serial.print("Could not connect to"); Serial.println(ssid);
    while(1) delay(500);
  }
  Serial.print("Ready! Use ");
  Serial.print(WiFi.localIP());
  Serial.println(" to access client");
}
 
 
void setup() {
  init_pin();
  Serial.begin(115200);
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(sub);
  initservo();
}
 
void loop() {
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  myservo1.write(servo1_pos+90);
  myservo2.write(servo2_f?110:80);
  delay(10);
}
