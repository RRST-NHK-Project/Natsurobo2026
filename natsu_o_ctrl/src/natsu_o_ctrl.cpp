/*
natsu_o_ctrl : ODrive手動制御（PS4）
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.

概要:
  PS4コントローラの □(SQUARE) ボタンで、3つのモータの速度指令を同時に切り替える。
  対象: odrv_a/axis0, odrv_a/axis1, odrv_b/axis0 の3軸。
  □を押すたびに:
      停止(0) -> 1回押し: SPEED_1 -> 2回押し: SPEED_2 -> 3回押し: SPEED_3 -> 4回押し: 停止(0)
  と巡回する（4回目で停止し、次の押下で1回目に戻る）。

構成:
  joy (sensor_msgs/Joy) を購読 -> 各 velocity_cmd (std_msgs/Float64) へ publish。
  natsu_odrive ノードがこのトピックを受けて各軸を回す。

注意:
  natsu_odrive には「0.5秒コマンドが来なければ0に落とす」ウォッチドッグがある。
  そのため本ノードはタイマーで現在の目標速度を常時再送する（押した時だけ送ると止まる）。
*/

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

// ===== 設定（必要に応じて変更）=====
#define SQUARE_BUTTON 3       // □ボタンの index（mc_2026.cpp と同じ）
#define SPEED_1 50.0          // 1回押しの速度 [turn/s]
#define SPEED_2 100.0         // 2回押しの速度 [turn/s]
#define SPEED_3 150.0         // 3回押しの速度 [turn/s]
#define PUBLISH_RATE_MS 20    // 常時再送の周期 [ms]（watchdog対策）
// ===================================

class OdriveManualControl : public rclcpp::Node
{
public:
    OdriveManualControl()
        : Node("natsu_o_ctrl")
    {
        // 速度指令を送る先（3軸すべてに同じ指令を送る）
        cmd_topics_ = {
            "/odrv_a/axis0/velocity_cmd",
            "/odrv_a/axis1/velocity_cmd",
            "/odrv_b/axis0/velocity_cmd",
        };

        // 初期状態: 停止(指示待ち)
        speed_state_ = 0;
        target_vel_ = 0.0;
        last_square_ = false;

        // joyノードの購読
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&OdriveManualControl::ps4_listener_callback, this, std::placeholders::_1));

        // 各軸への速度指令 publisher を作成
        for (const auto &topic : cmd_topics_)
        {
            cmd_pubs_.push_back(
                this->create_publisher<std_msgs::msg::Float64>(topic, 10));
        }

        // 常時再送タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&OdriveManualControl::publisher_timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "natsu_o_ctrl started. SQUARE(%d)で 停止->%.0f->%.0f->%.0f->停止 を巡回。対象=%zu軸",
                    SQUARE_BUTTON, SPEED_1, SPEED_2, SPEED_3, cmd_pubs_.size());
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ボタン配列が短い場合は無視（未接続・別マッピング対策）
        if (msg->buttons.size() <= static_cast<size_t>(SQUARE_BUTTON))
        {
            return;
        }

        bool SQUARE = (msg->buttons[SQUARE_BUTTON] > 0);

        // □の立ち上がり（押した瞬間）だけ状態を進める
        if (SQUARE && !last_square_)
        {
<<<<<<< HEAD
            // 状態遷移を明示的に記述（停止 -> SPEED_1 -> SPEED_2 -> SPEED_3 -> 停止）
            if (speed_state_ == 0)
            {
                speed_state_ = 1;
                target_vel_ = SPEED_1;
            }
            else if (speed_state_ == 1)
            {
                speed_state_ = 2;
                target_vel_ = SPEED_2;
            }
            else if (speed_state_ == 2)
            {
                speed_state_ = 3;
                target_vel_ = SPEED_3;
            }
            else
            {
                speed_state_ = 0;
=======
        speed_state_ = (speed_state_ + 1) % 3; // 0->1->2->0 の巡回
    
    RCLCPP_INFO(get_logger(),"SQUARE押下 -> 状態%d : %.1f turn/s " , speed_state_, target_vel_);  

    
       // 状態遷移を明示的に記述（停止 -> SPEED_1 -> SPEED_2 -> 停止）
        if (speed_state_ == 0)
            {
>>>>>>> 4959553 (ブラシレス)
                target_vel_ = 0.0;
            }
        else if (speed_state_   == 1)
            {
                target_vel_ = SPEED_1;
            }
        else if (speed_state_ == 2)
            {
                target_vel_ = SPEED_2;
            }     

        }        
    last_square_= SQUARE; // □の状態を更新    
    }
    

    void publisher_timer_callback()
    {
        // 現在の目標速度を全軸へ常時再送（watchdogを満たすため）
        std_msgs::msg::Float64 msg;
        msg.data = target_vel_;
        for (auto &pub : cmd_pubs_)
        {
            pub->publish(msg);
        }
    }

    std::vector<std::string> cmd_topics_;
    std::vector<rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> cmd_pubs_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

<<<<<<< HEAD
    int speed_state_;   // 0=停止, 1=SPEED_1, 2=SPEED_2, 3=SPEED_3
=======
    int speed_state_ = 0;   // 0=停止, 1=SPEED_1, 2=SPEED_2
>>>>>>> 4959553 (ブラシレス)
    double target_vel_; // 現在の目標速度 [turn/s]
    bool last_square_;  // □の前回状態（エッジ検出用）

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdriveManualControl>());
    rclcpp::shutdown();
    return 0;
}