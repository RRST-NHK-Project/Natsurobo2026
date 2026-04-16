#ifndef ZAKIOMNI_HPP
#define ZAKIOMNI_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include <chrono>
#include <iostream>
#include "sensor_msgs/msg/joy.hpp"
#include <algorithm>
#include <cmath>
#include <cstdlib>

#define cpr 8000//1回転あたり8000回と仮定
// スティックのデッドゾーン
#define DEADZONE_L 0.05
const double max_target_cps = 100.0; // 1秒あたりの最大回転数。速度管理はここをいじって
const double Kp  = 0.5; // P制御

using namespace std::chrono_literals;

class Zakicar : public rclcpp::Node {
 public:
    Zakicar();
      
    
 private:
    void encoderCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);  
    void PS4Callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void About_PID();

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr motor_pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr enc_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double measured_cps = 0.0;
    rclcpp::Time last;
    double zakiomni_v = 0.0;
    double target_v = 0.0;
    double err = 0.0;
    double rpm = 0.0;
    rclcpp::Time last_joy_time;
    bool joy_received = false;
    bool enc_received = false;
    int16_t zakistep = 0;
    uint16_t pre_enc = 0;
    int16_t enc_data_ = 0;
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    float LS_Y;
    //float LS_X;
    //float LS_Y;
    //float RS_X;
    // float RS_Y;
    // bool CROSS;
    // bool CIRCLE;
    // bool TRIANGLE;
    // bool SQUARE;
    //bool LEFT;
    //bool RIGHT;
    //bool UP;
    //bool DOWN;
    // bool L1;
    // bool R1;
    // float L2;
    //float R2;
    //bool SHARE;
    //bool OPTION;
    // bool PS;

    //bool L3;
    //bool R3;   
};

#endif 