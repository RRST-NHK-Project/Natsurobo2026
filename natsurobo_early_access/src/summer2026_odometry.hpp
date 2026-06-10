#ifndef SUMMER2026_ODOMETRY_HPP
#define SUMMER2026_ODOMETRY_HPP

#include <chrono>
#include <iostream>
#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <utility>
#include <mutex>

//ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/int16.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "std_msgs/msg/int32_multi_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

// 自作
#include "natsurobo_early_access/matrix.h"

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 2 // 送信先マイコンのID
#define INPUT_DEVICE_ID 3 // 受信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 17 // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意
#define cpr 8000               // 1回転あたり8000カウントと仮定
const float enc_max = 32767.0; // エンコーダーの最大値
#define opPI 3.1415926

using namespace std;


class Shivalian_control : public rclcpp::Node
{

public:
   Shivalian_control(uint8_t rx_device_id);

private:
   void publisher_position_callback();
   void sensor_callback_2(const std_msgs::msg::Int16MultiArray::SharedPtr msg);

   std::mutex read_only; 

   // オドメトリ設定(値をまだ変更してなくてデタラメになってる)
    static constexpr double ODOM_WHEEL_DIAMETER = 0.05;//あってる
    static constexpr double ODOM_WHEEL_RADIUS = ODOM_WHEEL_DIAMETER / 2.0;//自動的にあってる
    static constexpr double ODOM_WHEEL_CIRC = opPI * ODOM_WHEEL_DIAMETER;  //自動的に(ry
    static constexpr double ENCODER_RESOLUTION = 1024.0;
    static constexpr double ENC_TO_M = ODOM_WHEEL_CIRC / ENCODER_RESOLUTION;
    static constexpr double ODOM_LR_DISTANCE = 0.093;//0.0928がCAD上の値だけど
    static constexpr double ODOM_F_OFFSET = 0.335;

    static constexpr double ODOM_X_SCALE = 1.0;
    static constexpr double ODOM_Y_SCALE = 1.0;//ここらへんは宣言はしてあるけど全く使ってない
    static constexpr double ODOM_YAW_SCALE = 1.0;

   //命名規則:小文字は変数、大文字は行列、ii、jj、kkは単位はそれぞれx、y、z方向の単位ベクトルを表す行列、dの接頭辞が付くと変化量、r、Rは位置、_rはロボットを基準とした直行座標系
   rclcpp::Time current;
   rclcpp::Time last;
   float dt = 0.0;
   int16_t enc[3] = {0, 0, 0};
   int16_t last_enc[3] = {0, 0, 0};
   int16_t diff[3] = {0, 0, 0};
   int16_t ENC1;
   int16_t ENC2;
   int16_t ENC3;
   float rps[3] = {0.0, 0.0, 0.0};
   float v[3] = {0.0, 0.0, 0.0};
   float v_ = 0.0;   

   float q_rad = 0.0;
   float q_z = 0.0;
   float q_w = 0.0;

   float vx_r = 0.0;
   float vy_r = 0.0;
   float d_rad = 0.0;

   float dx_r = 0.0;
   float dy_r = 0.0;

   float dx = 0.0;
   float dy = 0.0;
   float d_yaw = 0.0;

   float point_Px = 0.0; 
   float point_Py = 0.0; 

   float yaw = 0.0;
   
   bool topic_received = false;

   //Matrix(mat.hで定義した行列)

   matrix V_wheel = matrix ({{0.0},
                             {0.0},
                             {0.0}}); //ロボットを原点とした基準での直交座標系の速度ベクトル

   matrix V_r = matrix ({{0.0},
                         {0.0},
                         {0.0}}); //ロボットを原点とした基準での直交座標系の速度ベクトル

   matrix dR_r = matrix ({{0.0},
                          {0.0},
                          {0.0}}); //ロボットを原点とした基準での直交座標系の変位ベクトル

   matrix dR = matrix ({{0.0},
                        {0.0},
                        {0.0}}); //ロボットを原点とした基準での直交座標系の変位ベクトル

   matrix R = matrix({{cos(yaw), -sin(yaw),0},
                      {sin(yaw), cos(yaw) ,0},
                      { 0,        0,       1}}); // 3×3のyaw回転行列(これでロボットを基準とした運動座標系A-ξηから固定座標系O-xyへの変換を行う)

   matrix FK = matrix ({{cos(-0*opPI/3), sin(-0*opPI/3), ODOM_LR_DISTANCE},//マイナスは単に車輪番号を時計回りに振ったせい
                        {cos(-2*opPI/3), sin(-2*opPI/3), ODOM_LR_DISTANCE},
                        {cos(-4*opPI/3), sin(-4*opPI/3), ODOM_LR_DISTANCE}}); //逆運動学における変換行列

   matrix FK_inv = FK.inv(); //mat.cppで逆行列へ(順運動学における変換行列)

   uint8_t rx_device_id_;
   uint8_t device_id_;

   rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
   rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sensor_sub_2;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
   rclcpp::TimerBase::SharedPtr timer_;
};

#endif