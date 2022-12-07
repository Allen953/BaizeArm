// Deng's FOC 开环位置控制例程 测试库：SimpleFOC 2.1.1 测试硬件：灯哥开源FOC V1.0
// 串口中输入"T+数字"设定两个电机的位置，如设置电机转到到180度，输入 "T3.14"（弧度制的180度）
// 在使用自己的电机时，请一定记得修改默认极对数，即 BLDCMotor(7) 中的值，设置为自己的极对数数字
// 程序默认设置的供电电压为 7.4V,用其他电压供电请记得修改 voltage_power_supply , voltage_limit 变量中的值
//               FOC_0.2  Baize_Foc
// IN1     pwm1     9        17
// IN2     pwm2     6        18
// IN3     pwm3     5        19
// INH1   enable1   8        21
// INH2   enable2   7        22
// INH3   enable3   4        23
// 电机极对数:11            减速比:11
//in-line current sense - phase 1/A 35
//in-line current sense - phase 1/C 34
// ESP32  iic接口  SCL:25   SDA:26
// AS5600 iic地址: 0x36
//I2Cone.begin(sda, scl, frequency); 


#include "WiFi.h"
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <rosserial_arduino/Adc.h>
#include <robot_msg/robotarm_7dof_jointstate.h>
#include <Wire.h>
#include <SimpleFOC.h>

#define rec 2.95
#define dir 1

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
TwoWire I2Cone = TwoWire(0);

BLDCMotor motor = BLDCMotor(11);
BLDCDriver3PWM driver = BLDCDriver3PWM(17, 19, 18, 21, 22, 23);


//目标位置
float target_position = 0;

//设置AP信息(SSID和PASSWORD)
const char* ssid = "S725";
const char* password = "s725s725";

IPAddress server(192, 168, 1, 108); // ip of your ROS server
IPAddress ip_address;
int status = WL_IDLE_STATUS;

WiFiClient client;
 
class WiFiHardware {
 
  public:
  WiFiHardware() {};
 
  void init() {
    // do your initialization here. this probably includes TCP server/client setup
    client.connect(server, 11412);
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

void JointStateCallback(const robot_msg::robotarm_7dof_jointstate& robotarm_joint) {
  motor.loopFOC();
  motor.move(robotarm_joint.position[2]*10+rec);


  Serial.println(sensor.getAngle());
  // We can now plot text on screen using the "print" class
  delay(1);
}
 
 
std_msgs::String str_msg;
ros::Publisher chatter("chatter11", &str_msg);
ros::Subscriber<robot_msg::robotarm_7dof_jointstate> subjoint("robotarm_joint", &JointStateCallback);

ros::NodeHandle_<WiFiHardware> nh;
char hello[20] = "Joint3 alive!";
 
 
void setupWiFi()
{
  WiFi.setHostname("BaizeArm-j3");
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
  I2Cone.begin(26, 25, 400000); 
  sensor.init(&I2Cone);
  //连接motor对象与传感器对象
  motor.linkSensor(&sensor);
  
  Serial.begin(115200);
  //供电电压设置 [V]
  driver.voltage_power_supply = 12.0;
  driver.init();

  //连接电机和driver对象
  motor.linkDriver(&driver);
  
  //FOC模型选择
  motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
  //运动控制模式设置
  motor.controller = MotionControlType::angle;

  //速度PI环设置
  motor.PID_velocity.P = 0.2;
  motor.PID_velocity.I = 20;
  //角度P环设置 
  motor.P_angle.P = 20;
  //最大电机限制电机
  motor.voltage_limit = 12.0;
  
  //速度低通滤波时间常数
  motor.LPF_velocity.Tf = 0.01;

  //设置最大速度限制
  motor.velocity_limit = 60;

  motor.useMonitoring(Serial);
  
  //初始化电机
  motor.init();
  //初始化 FOC
  motor.initFOC();

  Serial.println(F("Motor ready."));
  
  setupWiFi();
  delay(2000);
  nh.initNode();
  nh.advertise(chatter);
  nh.subscribe(subjoint);

  delay(100);

  
  Serial.println(F("Set the target velocity using serial terminal:"));

  _delay(1000);
}
 
void loop() {
  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
  delay(5);
}
