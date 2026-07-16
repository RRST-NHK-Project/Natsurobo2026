/*
natsu_o_ctrl : ODrive手動制御（PS4）
Copyright (c) 2026 RRST-NHK-Project. All rights reserved.

概要:
  PS4コントローラの □(SQUARE) ボタンで、ODrive(axis0)の速度指令を切り替える。
  □を押すたびに:
      停止(0) -> 1回押し: SPEED_1 -> 2回押し: SPEED_2 -> 3回押し: 停止(0) -> ...
  と巡回する。

構成:
  joy (sensor_msgs/Joy) を購読 -> /odrv_a/axis0/velocity_cmd (std_msgs/Float64) へ publish。
  natsu_odrive ノードがこのトピックを受けて axis0 を回す。

注意:
  natsu_odrive には「0.5秒コマンドが来なければ0に落とす」ウォッチドッグがある。
  そのため本ノードはタイマーで現在の目標速度を常時再送する（押した時だけ送ると止まる）。
*/

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float64.hpp"

// ===== 設定（必要に応じて変更）=====
#define CMD_TOPIC "/odrv_a/axis0/velocity_cmd" // 送り先（今回は axis0 の1台のみ）
#define SQUARE_BUTTON 3                         // □ボタンの index（mc_2026.cpp と同じ）
#define SPEED_1 10.0                            // 1回押しの速度 [turn/s]
#define SPEED_2 30.0                           // 2回押しの速度 [turn/s]
#define PUBLISH_RATE_MS 20                      // 常時再送の周期 [ms]（watchdog対策）
// ===================================

class OdriveManualControl : public rclcpp::Node
{
public:
    OdriveManualControl()
        : Node("natsu_o_ctrl")
    {
        // 初期状態: 停止(指示待ち)
        speed_state_ = 0;
        target_vel_ = 0.0;
        last_square_ = false;

        // joyノードの購読
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&OdriveManualControl::ps4_listener_callback, this, std::placeholders::_1));

        // ODriveへの速度指令 publisher
        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>(CMD_TOPIC, 10);

        // 常時再送タイマー
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&OdriveManualControl::publisher_timer_callback, this));

        RCLCPP_INFO(get_logger(),
                    "natsu_o_ctrl started. SQUARE(%d)で 停止->%.0f->%.0f->停止 を巡回。送り先=%s",
                    SQUARE_BUTTON, SPEED_1, SPEED_2, CMD_TOPIC);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        // ボタン配列が短い場合は無視（未接続・別マッピング対策）
        if (msg->buttons.size() <= static_cast<size_t>(SQUARE_BUTTON))
        {
            return;
        }

        bool SQUARE = msg->buttons[SQUARE_BUTTON];

        // □の立ち上がり（押した瞬間）だけ状態を進める
        if (SQUARE && !last_square_)
        {
            // 状態遷移を明示的に記述（停止 -> SPEED_1 -> SPEED_2 -> 停止）
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
            else
            {
                speed_state_ = 0;
                target_vel_ = 0.0;
            }

            RCLCPP_INFO(get_logger(),
                        "SQUARE押下 -> 状態%d : %.1f turn/s", speed_state_, target_vel_);
        }

        last_square_ = SQUARE; // 前回状態を更新
    }

    void publisher_timer_callback()
    {
        // 現在の目標速度を常時再送（watchdogを満たすため）
        std_msgs::msg::Float64 msg;
        msg.data = target_vel_;
        cmd_pub_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    int speed_state_;   // 0=停止, 1=SPEED_1, 2=SPEED_2
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
