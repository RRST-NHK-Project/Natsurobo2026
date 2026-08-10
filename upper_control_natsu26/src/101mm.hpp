#ifndef __101MM_HPP__
#define __101MM_HPP__


#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

//クレーンゲームパッチ(小うなぎ捕獲機構パッチ)を当てるかどうか
// #define CRANEGAME

// 以下マイコンに合わせて設定
#define TX_DEVICE_ID 2 // 送信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 17 // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

#define drive_mode (L1_count % 2 == 0)       // L1を押していないときはドライブモード
#define get_eel_mode (L1_count % 2 == 1) // L1を押しているときは捕獲モード

class unaginobori2026 : public rclcpp::Node
{
public:
    unaginobori2026(uint8_t tx_device_id);

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
    void publisher_timer_callback();
    // natsu_auto の CLIMB 状態から /climb/start を受けて自動昇降を実行する
    void climb_start_callback(const std_msgs::msg::Bool::SharedPtr msg);

    #if defined(CRANEGAME)
        int cranegame_servo();
        int cranegame_motor();
        bool cranegame_tr();
    #endif

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;
    int L1_count = 0;
    int CIRCLE_count = 0;
    
    int CROSS_count = 0;
    int SQUARE_count = 0;
    bool last_CROSS = false;
    bool last_SQUARE = false;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 自動昇降(CLIMB): /climb/start 受信でシリンダ上げ、raise_sec_ 秒後に /climb/done を返す
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr climb_start_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr climb_done_pub_;
    bool climbing_ = false;                 // 自動昇降シーケンス実行中か
    rclcpp::Time climb_start_time_;         // シーケンス開始時刻
    double raise_sec_ = 2.0;                // シリンダ上げ保持時間[s](param)

    std::vector<int16_t> data_;

    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        bool CIRCLE;
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        bool L1;
        // bool R1 = msg->buttons[5];

        float L2_DIGITAL;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;
        bool last_CIRCLE = false;
        bool last_L1 = false;
        // bool last_R1 = false;

        #if defined(CRANEGAME)
            bool TRIANGLE;
            bool CROSS;
            bool SQUARE;
        #endif

};

#if defined(CRANEGAME)
    #include "cranegame.hpp"
#endif

#endif 