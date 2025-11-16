/*
===========================免责声明===========================
1、该项目只适用于学习研究飞行原理，使用者不得将其用违规飞行、侵犯他人权益或其他违反国家法律法规、公序良俗的场景，否则一切法律责任由使用者自行承担。
2、项目处于开源学习阶段，可能存在未被发现的安全漏洞、性能缺陷或逻辑瑕疵，使用及飞行过程中可能出现失控、坠机等不可预知的风险。
3、使用者在进行测试或使用前，必须充分评估风险，严格遵守相关法律法规及场地管理规定，仅在具备完整安全防护措施的封闭式空间内操作。测试前需确认场地无人员、易燃易损物品及其他障碍物，并做好个人防护。
4、因使用本项目（包括但不限于直接使用、擅自修改代码 / 硬件、违规操作等）导致的任何人身伤害、财产损失、第三方索赔及相关法律责任，均由使用者独立承担全部责任，项目作者及所有贡献者不承担任何直接、间接或连带的赔偿责任及法律责任。
5、使用者下载、克隆、使用本项目的任何资源，即视为已充分知晓本声明全部条款，理解并接受所有潜在风险，若不同意本声明请立即停止获取和使用本项目相关资源。





code by:ZhuHenCai  地球号:137967547  Email:137967547@qq.com
============================================================
*/


/*
des:程序使用arduino 开发编译，mpu6050使用的是Adafruit MPU6050库

代码也是作者一路学习、一路测试、一路不停地修改过来的，改来改去有点屎山，比较乱，介意的可直接路过。
整体还是相对简单，姿态计算、PID计算上使用原始简单的欧拉角进行计算，没有使用矩阵、四元素、ROTS。。。，相对适用飞行原理初学者入门。

原来使用的是GY-87三合一传感器，后面发现第一次弄，读取气压、磁力计后DT各种抖动，比较折腾，直接改成了MPU6050,所以项目中会有一些残余的相关代码片段，可直接删掉

飞行效果：https://space.bilibili.com/3632307692898445


如果使用的是无刷电机，直接把PWM信号相关的代码改一下就可以了。

*/





#include <Wire.h>
#include <Adafruit_MPU6050.h> 
#include <Adafruit_Sensor.h> 
#include <WiFi.h>       
#include <WiFiUdp.h>    
#include <cmath>   
#include <vector> 
#include <Preferences.h> 
#include <ArduinoJson.h>
#include "driver/rtc_io.h"
#include <Preferences.h>


//------------ELRS/CRSF -------------

#include "HardwareSerial.h"
// 串口2
#define CRSF_SERIAL_PORT Serial2
// RX 引脚  
#define CRSF_RX_PIN 16 
//TX 引脚             
#define CRSF_TX_PIN 17       
#define CRSF_BAUDRATE 420000
//16个通道
#define CRSF_CHANNEL_COUNT 16 
//失联时间
#define CRSF_FAILSAFE_THRESHOLD 500

#define CRSF_SYNC_BYTE 0xC8
#define CRSF_FRAME_TYPE_RC_CHANNELS 0x16

// 控制模式
enum ControlMode {
  CONTROL_MODE_WIFI,
  CONTROL_MODE_ELRS
};
//默认wifi
ControlMode currentControlMode = CONTROL_MODE_WIFI; 
bool crsf_failsafe = true;
unsigned long last_crsf_packet_time = 0;
unsigned int crsf_channel_data[CRSF_CHANNEL_COUNT];

//较准数据
/*
16:24:08.123 ->   Offset X/Y/Z: 446.50, -65.50, -489.50
16:24:08.123 ->   Scale X/Y/Z: 1.01, 1.00, 0.99

12:54:03.113 ->   Offset X/Y/Z: 448.50, -71.50, -533.00
12:54:03.113 ->   Scale X/Y/Z: 1.01, 1.00, 0.99

16:24:09.614 -> 开始校准陀螺仪... 请保持无人机绝对静止！
16:24:09.614 -> 正在采样...
16:24:19.631 -> 陀螺仪校准完成。
16:24:19.631 -> X 轴偏移 (deg/s): -2.68
16:24:19.631 -> Y 轴偏移 (deg/s): 1.61
16:24:19.631 -> Z 轴偏移 (deg/s): -0.68
*/

// --- 六面校准数据
Preferences preferences;
float accel_x_offset_saved = 448.50f;
float accel_y_offset_saved = -71.50f;
float accel_z_offset_saved = -533.00f;
float accel_x_scale_saved = 1.01f;
float accel_y_scale_saved = 1.0f;
float accel_z_scale_saved = 0.99f;


//地面水平问题，较准
float takeoff_trim_roll = 0.0f;
float takeoff_trim_pitch = 0.0f;
// =========================================================




//  基础配置 
#define MOTOR1_PIN 4
#define MOTOR2_PIN 27
#define MOTOR3_PIN 33 
#define MOTOR4_PIN 19 

//PWM频率、分辨率、最大值设置
const int PWM_FREQ = 1000;  
const int PWM_RESOLUTION = 12;  
const int MIN_PWM_SIGNAL = 0; 
const int MAX_PWM_SIGNAL = 1000; 
const int MAX_PWM_VALUE = (1 << PWM_RESOLUTION) - 1; 
const int MOTOR_STOP_PWM = 0;


//--------wifi 控制----------
 // WiFi名称
const char* ssid = "zhuhencai";
   // WiFi密码
const char* password = "12345678";
WiFiUDP udp;  
// 本地监听端口                 
unsigned int localUdpPort = 4210; 
//缓冲区
char incomingPacket[255];  
char replyPacket[] = "OK"; 
const int UDP_RX_JSON_BUFFER_SIZE = 128;
const int UDP_TX_JSON_BUFFER_SIZE = 256;


//logo相关参数
#define LOG_RATE 10 
#define LOG_DURATION 15 
#define LOG_SIZE LOG_RATE * LOG_DURATION
int logIndex = 0;   
bool loggingEnabled = true; 


//  操控
const float RC_RATE_EXPO = 0.55f; 
// I2C 地址 
#define MPU6050_ADDR 0x68
// 最大控制角度/速率
const float MAX_TILT_ANGLE = 25.0; 
const float MAX_YAW_RATE = 30.0; 

//  传感器对
Adafruit_MPU6050 mpu;
bool mpu_initialized = false;


// 传感器数据
float accX = 0.0f, accY = 0.0f, accZ = 0.0f;  
float gyroX_rad = 0.0f, gyroY_rad = 0.0f, gyroZ_rad = 0.0f; 
float gyroX_cal = 0.0f, gyroY_cal = 0.0f, gyroZ_cal = 0.0f;
float roll = 0.0f, pitch = 0.0f, yaw = 0.0f; 



//微分抖动很大，加个低通，坏处是反应变慢
float filtered_roll_D_output = 0.0f;
float filtered_pitch_D_output = 0.0f;
float filtered_yaw_D_output = 0.0f;

float angle_pitch = 0.0f;
float angle_roll = 0.0f;
float angle_yaw = 0.0f;

//0点水平偏移
float gyroXoffset_rad = -1.04f;
float gyroYoffset_rad = 0.67f;
float gyroZoffset_rad = -0.14f;
//初始安装偏差
float pitch_offset =0.0f;
float roll_offset = 0.0f;//-8.1f;//左飞矫正
float yaw_offset = 0.0f;
//弧度转度
#define MAG_DECLINATION -2.8f * M_PI / 180.0f

//  高度移动平均滤波器 
const int ALTITUDE_FILTER_SIZE = 10;
std::vector<float> altitude_history;
float altitude_sum = 0.0f;

//PID 控制器参数

/*
// 角度环PID参数 - 降低微分增益，调整积分限幅
float kp_roll_angle = 3.8f, ki_roll_angle = 0.03f, kd_roll_angle = 0.2f;
float kp_pitch_angle = 3.8f, ki_pitch_angle = 0.03f, kd_pitch_angle = 0.2f;
float kp_yaw_angle = 3.8f, ki_yaw_angle = 0.02f, kd_yaw_angle = 0.05f;

// 角速度环PID参数 - 增强比例控制
float kp_roll_rate = 1.4f, ki_roll_rate = 0.01f, kd_roll_rate = 0.1f;
float kp_pitch_rate = 1.4f, ki_pitch_rate = 0.01f, kd_pitch_rate = 0.1f;
float kp_yaw_rate = 1.5f, ki_yaw_rate = 0.01f, kd_yaw_rate = 0.02f;
不错的一组
float kp_roll_angle =2.6f, ki_roll_angle = 0.06f, kd_roll_angle = 0.05f;
float kp_pitch_angle = 2.6f, ki_pitch_angle = 0.06f, kd_pitch_angle = 0.05f;
float kp_yaw_angle = 2.8f, ki_yaw_angle = 0.05f, kd_yaw_angle = 0.05f;

float kp_roll_rate = 2.8f, ki_roll_rate = 0.12f, kd_roll_rate = 0.07f;
float kp_pitch_rate = 2.8f, ki_pitch_rate = 0.12f, kd_pitch_rate = 0.07f;
float kp_yaw_rate = 2.8f, ki_yaw_rate = 0.1f, kd_yaw_rate = 0.06f;

*/

float kp_roll_angle =2.6f, ki_roll_angle = 0.02f, kd_roll_angle = 0.05f;
float kp_pitch_angle = 2.6f, ki_pitch_angle = 0.02f, kd_pitch_angle = 0.05f;
float kp_yaw_angle = 2.8f, ki_yaw_angle = 0.02f, kd_yaw_angle = 0.05f;

float kp_roll_rate = 2.8f, ki_roll_rate = 0.01f, kd_roll_rate = 0.07f;
float kp_pitch_rate = 2.8f, ki_pitch_rate = 0.01f, kd_pitch_rate = 0.07f;
float kp_yaw_rate = 2.8f, ki_yaw_rate = 0.01f, kd_yaw_rate = 0.06f;


//PID 运算变量

float roll_angle_error = 0.0f, pitch_angle_error = 0.0f, yaw_angle_error = 0.0f; 
float roll_angle_integral = 0.0f, pitch_angle_integral = 0.0f, yaw_angle_integral = 0.0f; 
float prev_roll_angle = 0.0f, prev_pitch_angle = 0.0f; 
float target_roll_rate = 0.0f, target_pitch_rate = 0.0f,target_yaw_rate = 0.0f;
float roll_rate_error = 0.0f, pitch_rate_error = 0.0f, yaw_rate_error = 0.0f; 
float roll_rate_integral = 0.0f, pitch_rate_integral = 0.0f, yaw_rate_integral = 0.0f; 
float roll_rate_derivative = 0.0f, pitch_rate_derivative = 0.0f, yaw_rate_derivative = 0.0f; 
float  prev_gyro_z_filtered=0.0f,prev_gyro_y_filtered=0.0f,prev_gyro_x_filtered=0.0f;
float roll_pid_output = 0.0f, pitch_pid_output = 0.0f, yaw_pid_output = 0.0f; 


//外环上次角度误差
float roll_angle_prev_error = 0.0f, pitch_angle_prev_error = 0.0f, yaw_angle_prev_error = 0.0f;
//内环上次角速度误差
float pitch_rate_prev_error, roll_rate_prev_error = 0.0, yaw_rate_prev_error = 0.0;


//内环角速度低通融合
float gyro_alpha = 0.8f;
float gyro_x_filtered = 0.0f, gyro_y_filtered = 0.0f, gyro_z_filtered =0.0f;

//  飞行状态
enum FlightMode { FLIGHT_MODE_DISARMED, FLIGHT_MODE_ARMED, FLIGHT_MODE_FLIPPING, FLIGHT_MODE_TURTLE };
FlightMode currentFlightMode = FLIGHT_MODE_DISARMED;
bool isFlippedOver = false;

//----------遥控 ----------
float rc_throttle_input = 0.0f; 

float rc_raw_roll_input = 0.0f; 
float rc_raw_pitch_input = 0.0f;
float rc_raw_yaw_input = 0.0f;
float rc_target_roll_angle = 0.0f; 
float rc_target_pitch_angle = 0.0f; 
float rc_target_yaw_rate = 0.0f;


// 翻滚实现，参考AI给的思路，测试废了几个机架，觉得这思路有严重问题，未实现
enum FlipDirection { FLIP_NONE, FLIP_FRONT, FLIP_BACK, FLIP_LEFT, FLIP_RIGHT };
FlipDirection currentFlipDirection = FLIP_NONE; // 翻滚
unsigned long flipStartTime = 0; // 翻滚开始时间 (ms)
const int FLIP_DURATION_MS = 600;       // 翻滚预计时间 (ms)
const int FLIP_BOOST_DURATION_MS = 80;  // 初始油门增压时间 (ms)
const int FLIP_ROTATE_DURATION_MS = 220;// 旋转时间 (ms)
const int FLIP_CUT_THROTTLE_START_MS = 80;//翻滚减油门的开始时间
const int FLIP_CUT_THROTTLE_END_MS = 180; // 翻滚中削减油门的结束时间
const float FLIP_RATE_TARGET = 1000.0f;  // 翻滚目标角速率
const float FLIP_THROTTLE_BOOST = 1.25f; //  翻滚前油门增加倍数 
const float FLIP_THROTTLE_CUT_FACTOR= 0.4f;// 翻滚中油门削减系数
const float MIN_FLIP_ALTITUDE = 1.5f;    //  允许翻转的最低高度

//PID计算后分配的油门
int m1_pwm, m2_pwm, m3_pwm, m4_pwm;

// 反乌龟
unsigned long turtleMotorPulseTime = 0;   
const int TURTLE_PULSE_DURATION_MS = 150; 
const int TURTLE_MOTOR_POWER_PWM = 350;  


//  其他全局变量 
unsigned long last_loop_time = 0; 
float loop_time_s = 0.0015f; 
bool isPrintLog=false;
//气压计 兹力计多少循环执行一次
unsigned long loop_counter = 0; 
unsigned long last_udp_command_time = 0; 
const unsigned long UDP_TIMEOUT_MS = 4000;//原2000

// MPU6050 寄存器地址
#define MPU6050_RA_USER_CTRL    0x6A
#define MPU6050_RA_INT_PIN_CFG  0x37
#define MPU6050_USERCTRL_I2C_MST_EN_BIT 5
#define MPU6050_INTCFG_I2C_BYPASS_EN_BIT 1
uint32_t LoopTimer;


void setup() {
  Serial.begin(115200);
  Serial.println("无人机初始化 ..."); // 版本号更新
  delay(10); // 等待串口通信稳定
  //查看重启原因
 // print_reset_reason_detailed();
  // 禁用掉电保护
  disableBrownOut();
  Wire.begin();
  Wire.setClock(400000);

  delay(2); 
  setupSensors();
/*  六面调较准后用，现调试读取后已经定为常量，正式后开放便于较准
 // --- 加载永久校准数据 ---
  preferences.begin("drone-calib", false);
  accel_x_offset_saved = preferences.getFloat("accelXOff", 0.0f);
  accel_y_offset_saved = preferences.getFloat("accelYOff", 0.0f);
  accel_z_offset_saved = preferences.getFloat("accelZOff", 0.0f);
  accel_x_scale_saved = preferences.getFloat("accelXScale", 1.0f);
  accel_y_scale_saved = preferences.getFloat("accelYScale", 1.0f);
  accel_z_scale_saved = preferences.getFloat("accelZScale", 1.0f);
  preferences.end();
  */
  Serial.println("已加载永久加速度计校准数据:");
  Serial.print("  Offset X/Y/Z: "); Serial.print(accel_x_offset_saved); Serial.print(", "); Serial.print(accel_y_offset_saved); Serial.print(", "); Serial.println(accel_z_offset_saved);
  Serial.print("  Scale X/Y/Z: "); Serial.print(accel_x_scale_saved); Serial.print(", "); Serial.print(accel_y_scale_saved); Serial.print(", "); Serial.println(accel_z_scale_saved);

  delay(1000); //手动开机时，机身会抖动，这里设置延迟久一点
  calibrateGyro();
  delay(2); 
  setupPWM();
  delay(2); 

  // 设置ELRS并决定控制模式
  setupELRSAndDetermineControlMode();
  //没有ELRS 就接WIFI
  if (currentControlMode == CONTROL_MODE_WIFI) {
    setupWiFiUDP();
    Serial.println("初始化完成，等待UDP指令...");
    Serial.print("连接到WiFi AP: "); Serial.println(ssid);
    Serial.print("发送UDP指令到 IP: "); Serial.print(WiFi.softAPIP());
    Serial.print(" 端口: "); Serial.println(localUdpPort);
  } else {
    Serial.println("初始化完成，等待ELRS遥控信号...");
  }
  //LED灯
  pinMode(2, OUTPUT);
  rc_throttle_input = 0;
  currentFlightMode = FLIGHT_MODE_DISARMED;
  last_loop_time=micros();
  LoopTimer = last_loop_time;
} 



void loop() {


  unsigned long now = micros();
  float dt = (now - last_loop_time) / 1000000.0f;
  dt = constrain(dt, 0.0001f, 0.009f);  
  loop_time_s = dt;
  last_loop_time = now;

  readSensors();

  //3、计算当前姿态 低通融合
  calculateAttitude(); 
  // 选择控制模式输入源
  if (currentControlMode == CONTROL_MODE_ELRS) {
    // ELRS接收机
    readCRSF();
  } else {
    // WiFi UDP
    processUDP();
  }
  // 更新飞行状态，特技翻转、倒地翻转，特技后需要矫正       
  updateFlightMode();
  // 执行状态控制逻辑  
  runControlLoop();    
  safetyCheck(); 



  // 检查串口命令，执行对应命令
  if (Serial.available()) {
      char c = Serial.read();
      //计算安装偏移
      /*
      if (c == 'c') {
      //简单较准
      calibrateAndSaveAccelOffsets();
      }
*/
      if (c == 'm') {
        //六面较准
        performSixPointCalibration();
      }

      if (c == 'd') {  // 发送'd'字符转储日志
        dumpLog();
      } else if (c == 'e') {  // 发送'c'字符清除日志
        logIndex = 0;
        loggingEnabled = true;
        Serial.println("Log buffer cleared");
      }
  }

  //0.004秒执行一次，MPU6050 滤波延迟0.004*1000000 高频MPU6050不稳定 
  while (micros() - LoopTimer < (2000));
  {
    LoopTimer = micros();
  }
  // 循环计数 
  loop_counter++;
} 
