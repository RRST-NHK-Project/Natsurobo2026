#ifndef ZAKIOMNI_HPP
#define ZAKIOMNI_HPP


#include <chrono>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <utility>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp" //変な挙動したら削除して下さい。imuによる制御補助用。
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "std_msgs/msg/bool.hpp"   //自動化用（/auto/abort通知）
#include "std_msgs/msg/string.hpp" //自動化用（/auto/arbitration受信）
#include "geometry_msgs/msg/twist.hpp" //上に同じく。自動旋回用。

#define opPI 3.1415926

// 以下マイコンに合わせて設定
#define TX_DEVICE_ID 1 // 送信先マイコンのID
#define RX_DEVICE_ID 1 // 受信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 17 // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

//PIDのモード設定
#define Mode_normal
//#define Mode_custom

//PIDを行う場所の設定
#define PC
// #define ESP32

// スティックのデッドゾーン
#define DEADZONE_L 0.15
#define DEADZONE_R 0.15
#define cpr 8192              // 1回転あたり8000カウント（確認済）
const float enc_max = 32768.0; // エンコーダーの最大値

// 　よく調整する定数集(For Mabuchi 775 motor))
const float max_target_move_rps = 12.5; // 1秒あたりの最大回転数(移動方向)
const float max_target_yaw_rps = 8.5;  // 1秒あたりの最大回転数(回転方向)
const float Kp_yaw = 4.0;               // ヘディングロック P ゲイン [rps/rad] (要調整)
const float Kff = 0.0;                  // フィードフォワード（必要に応じて調整するつもりだったけどいらんかッた）
const float Kp = 7.0;                   // P制御//無負荷なら7.5あたり？負荷がかかると8,5でもいいかも
const float Ki = 1.35;                   // I制御
const float Kd = 0.1;                   // D制御(ただしめっちゃ振動するから封印中)
const float filter = 0.2;               // フィルタ係数（小さいほどスムーズらしい）
const float Imax = 45.0;                // I制御の蓄積の上限（必要に応じて調整）
const float motor_limit = 100.0;         // モーターの出力の上限（0~100で）
const int delta_power_limit = 40;       // 出力変化の上限
const float timeout = 1.0;

//ホイールごとの個別設定（customモード有効時）
const float Kff_[4] = {0.0,0.0,0.0,0.0};
const float Kp_[4] = {0.0,0.0,0.0,0.0};
const float Ki_[4] = {0.0,0.0,0.0,0.0};
const float Kd_[4] = {0.0,0.0,0.0,0.0};
const float filter_[4] = {0.0,0.0,0.0,0.0};
const float Imax_[4] = {0.0,0.0,0.0,0.0};

using namespace std::chrono_literals;

//zakiomni.hpp
class Zakicar : public rclcpp::Node
{
public:
   Zakicar(uint8_t tx_device_id, uint8_t rx_device_id);
   

private:
   void sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg);
   void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
   void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
   void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg); // 自動走行(調停実装済みで有効化)
   void publisher_timer_callback();
   void about_PID();
   void Timeout_check();
   void Shivangelion(); // ノード名表示兼デバック用関数

   uint8_t tx_device_id_;
   uint8_t rx_device_id_;
   uint8_t device_id_;

   rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
   rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_;
   rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
   rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
   // 自動走行(/cmd_vel)用
   rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
   std::atomic<bool> cmd_vel_received_{false};
   rclcpp::Time last_cmd_vel_time_;
   double cmd_vel_v_ref_ = 0.8;   // [m/s] ≒ 車輪周長×max_target_move_rps。要実測調整
   double cmd_vel_w_ref_ = 2.0;  // [rad/s] 全輪max_target_yaw_rpsで回った時の機体角速度。要実測調整
   double cmd_vel_min_ratio_v_ = 0.08;
   double cmd_vel_min_ratio_w_ = 0.035;
   double joy_override_th_ = 0.6;

   // 調停(/auto/arbitration): "auto"でcmd_velを受け付け、"manual"でjoy専属に戻る。
   // AUTO中にスティックが倒されたら即座にmanualへ落とし、/auto/abortでFSMに通知する。
   rclcpp::Subscription<std_msgs::msg::String>::SharedPtr arbitration_sub_;
   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr abort_pub_;
   std::atomic<bool> auto_mode_{false};
   rclcpp::TimerBase::SharedPtr timer_;

   // IMU ヘディングロック
   double imu_yaw_{0.0};
   double target_yaw_{0.0};
   bool have_imu_{false};

   std::vector<int16_t> data_;
   std::vector<int16_t> last_data_ = {0, 0, 0, 0};

   rclcpp::Time current;
   rclcpp::Time last;

   float target_v[4] = {0.0, 0.0, 0.0, 0.0};
   float last_target_v[4] = {0.0, 0.0, 0.0, 0.0};
   float err[4] = {0.0, 0.0, 0.0, 0.0};
   float last_err[4] = {0.0, 0.0, 0.0, 0.0};
   float err_diff[4] = {0.0, 0.0, 0.0, 0.0};
   float err_sum[4] = {0.0, 0.0, 0.0, 0.0};
   float rps[4] = {0.0, 0.0, 0.0, 0.0};
   float FF[4] = {0.0, 0.0, 0.0, 0.0};
   float P[4] = {0.0, 0.0, 0.0, 0.0}, I[4] = {0.0, 0.0, 0.0, 0.0}, D[4] = {0.0, 0.0, 0.0, 0.0};
   float motor_power[4] = {0.0, 0.0, 0.0, 0.0};

   float target_rpm[4] = {0.0, 0.0, 0.0, 0.0};

   float dt = 0.0;
   float radian = 0.0;

   // フラグ関連の変数
   double enc_blank_time = 0.0;
   double joy_blank_time = 0.0;
   rclcpp::Time last_joy_time;
   rclcpp::Time last_enc_time;
   std::atomic<bool> joy_received{false};
   std::atomic<bool> enc_received{false};
   std::atomic<bool> shivangelion_activated{false};
   std::atomic<bool> rps_count[4] = {false, false, false, false}; // エンコーダが回転しているかのフラグ（trueなら回転していると判断）
   int8_t count_true = 0;                                         // rps_countの中でtrueの数を数えるための変数
   int8_t count_false = 0;                                        // rps_countの中でfalseの数を数えるための変数
   int8_t doubt_enc_num = 0;                                      // 疑わしきエンコーダの番号を特定するための変数
   int8_t rps_num_count = 0;                                      // 回転しているエンコーダの数をとる（3なら空転している可能性が高い）
   int16_t diff[4] = {0, 0, 0, 0};
   uint16_t last_enc[4] = {0, 0, 0, 0};

   // コントローラーの入力を取得、使わない入力はコメントアウト推奨
   float LS_X;
   float LS_Y;
   float RS_X;
   // float RS_Y;
   // bool CROSS;
   // bool CIRCLE;
   // bool TRIANGLE;
   // bool SQUARE;

   // bool LEFT;
   // bool RIGHT;
   // bool UP;
   // bool DOWN;

   // bool L1;
   // bool R1;

   // float L2_DIGITAL;
   float R2_DIGITAL;

   // bool L2;
   // bool R2;

   // bool SHARE;
   // bool OPTION;
   // bool PS;

   // bool L3;
   // bool R3;

   // static bool last_option = false;
   // static bool option_latch = false;

   // static bool last_share = false;
   // static bool share_latch = false;

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
#if (defined(Mode_custom) + defined(Mode_normal)) !=1
#error "Please choose "ONE" mode!!!"
#endif

#if (defined(PC) + defined(ESP32)) !=1
#error "Please choose "ONE" PID location!!!"
#endif

#endif