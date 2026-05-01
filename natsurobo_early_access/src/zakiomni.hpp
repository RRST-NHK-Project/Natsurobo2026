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
#define opPI 3.1415926
#define zaki -1
// 以下マイコンに合わせて設定
#define TX_DEVICE_ID 1 // 送信先マイコンのID
#define RX_DEVICE_ID 1 // 受信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 17 // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3


//　よく調整する定数集(For Mabuchi 775 motor))
#define cpr 8192//1回転あたり8000カウントと仮定
const double max_target_cps = 8.0; // 1秒あたりの最大回転数
const double Kp  = 10.0;//P制御(必要に応じて調整)
const double Ki = 0.5; // I制御（必要に応じて調整）
const double Imax = 30.0; // I制御の蓄積の上限（必要に応じて調整）
const double motor_limit = 80.0; // モーターの出力の上限（0~100で）
const int delta_power_limit = 6;// 出力変化の上限
const double enc_max = 32767.0; // エンコーダーの最大値

using namespace std::chrono_literals;

class Zakicar : public rclcpp::Node {
 public:
    Zakicar(uint8_t tx_device_id, uint8_t rx_device_id);
      
    
 private:
    void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);  
    void ps4_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publisher_timer_callback();
    void about_PID();
    void Shivangelion();//ノード名表示兼デバック用関数
    void safety_check();

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sns_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
    
    rclcpp::Time current = this->get_clock()->now();
    rclcpp::Time last = this->get_clock()->now();

    double target_v[4] = {0.0, 0.0, 0.0, 0.0 };
    double err[4] = {0.0, 0.0, 0.0, 0.0};
    double err_sum[4] = {0.0, 0.0, 0.0, 0.0};
    double zakirps[4] = {0.0, 0.0, 0.0, 0.0};
    double P[4] = {0.0, 0.0, 0.0, 0.0}, I[4] = {0.0, 0.0, 0.0, 0.0}, motor_power[4] = {0.0, 0.0, 0.0, 0.0};
    double dt = 0.0; 
    double radian = 0.0;

    //フラグ関連の変数
    rclcpp::Time last_joy_time = this->get_clock()->now();
    bool joy_received = false;
    bool enc_received = false;
    bool ok = false;
    bool shivangelion_activated = false;
    int zakipow[4] = {0, 0, 0, 0};
    int last_zakipow[4] = {0, 0, 0, 0};
    int32_t diff32[4] = {0, 0, 0, 0};
    int16_t diff[4] = {0, 0, 0, 0};
    int32_t now_enc[4] = {0, 0, 0, 0};
    int32_t pre_enc32[4] = {0, 0, 0, 0};//エンコーダの値の計算用
    uint16_t pre_enc[4] = {0, 0, 0, 0};
    double angle = 0.0;

    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    float LS_X;
    float LS_Y;
    //float RS_X;
    //float RS_Y;
    //bool CROSS;
    //bool CIRCLE;
    //bool TRIANGLE;
    //bool SQUARE;
    //bool LEFT;
    //bool RIGHT;
    //bool UP;
    //bool DOWN;
    //bool L1;
    //bool R1;
    //float L2;
    //float R2;
    //bool SHARE;
    //bool OPTION;
    //bool PS;
    //bool L3;
    //bool R3;   

   int16_t ENC1 = 0;
   int16_t ENC2 = 0;
   int16_t ENC3 = 0;
   int16_t ENC4 = 0;
        // int16_t ENC5 = msg->data[5];
        // int16_t ENC6 = msg->data[6];
        // int16_t ENC7 = msg->data[7];
        // int16_t ENC8 = msg->data[8];

        // int16_t SW1 = msg->data[9];
        // int16_t SW2 = msg->data[10];
        // int16_t SW3 = msg->data[11];
        // int16_t SW4 = msg->data[12];
        // int16_t SW5 = msg->data[13];
        // int16_t SW6 = msg->data[14];
        // int16_t SW7 = msg->data[15];
        // int16_t SW8 = msg->data[16];
};

#endif 