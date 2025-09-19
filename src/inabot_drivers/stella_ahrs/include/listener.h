#pragma once
#include <iostream>
#include <stdlib.h>
#include <tf2/LinearMath/Quaternion.h>
#include <math.h>
#include <memory>
#include <chrono> 
#include <string>
#include <pthread.h>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "MwSerial.hpp"

// ----------------------------
// 상수 정의
// ----------------------------
#define DEG2RAD(a) ((a) * (M_PI / 180.0f))
#define COS(a) cos(DEG2RAD(a))
#define SIN(a) sin(DEG2RAD(a))

#define DeviceID 0x01
#define STX 0x02
#define ETX 0x03
#define Command 0xF0

#define ACC 0x33
#define GYO 0x34
#define DEG 0x35

// ----------------------------
// 전역 변수
// ----------------------------
extern char recv_buf[8];
extern long id;
extern int length;
extern bool run;
extern std::shared_ptr<sensor_msgs::msg::Imu> imu;

// ----------------------------
// 함수 선언
// ----------------------------
void* AHRS_thread(void* arg);
