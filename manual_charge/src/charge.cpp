#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdint>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/string.hpp>

#define OUTPUT_DEVICE_ID 0x04 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID 0x04  // 入力マイコン（マイクロスイッチやエンコーダ）のID
#define TX16NUM 24            // 送信データ数
#define RX16NUM 17            // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

#define LED_SLOT 9 // LED用のスロット先頭。data_[9]〜data_[12] の4本ぶん使う

// 装填モーターの出力値(MDのPWM指令, -255〜255)。OPTIONで ON/OFF する。
constexpr int16_t CHARGE_MOTOR_POWER = 50;

// 装填モーターを繋ぐMDの番号の既定値。
// ID4基板は data_[1]〜data_[4] が MD1〜MD4 にそのまま対応している
// (ファーム側 pin_ctrl_task.cpp の MD_Output / NATSU_ID4_init は4本とも初期化済み)。
// 現物の基板はMD1が壊れているのでMD2を使っている。配線を変えたら
//   ros2 run manual_charge charge --ros-args -p md_number:=3
// で追従できる(再ビルド不要)。launchからは manual_launch_natsu_2026.launch.py で指定。
constexpr int DEFAULT_MD_NUMBER = 2;
 
// 使用するモーターの選択
#define MODE_MABUCHI
// #define MODE_BLDC

class ChargeNode : public rclcpp::Node
{
public:
  ChargeNode()
  : Node("charge_node")
  {
    data_.assign(TX16NUM, 0);

    // 使用するMDの番号(1〜4)。data_[md_number] がそのMDの出力になる。
    const int md_number = this->declare_parameter<int>("md_number", DEFAULT_MD_NUMBER);
    if (md_number < 1 || md_number > 4) {
      RCLCPP_ERROR(this->get_logger(),
        "md_number=%d は範囲外です(1〜4)。既定の MD%d を使います。",
        md_number, DEFAULT_MD_NUMBER);
      md_slot_ = DEFAULT_MD_NUMBER;
    } else {
      md_slot_ = md_number;
    }
    RCLCPP_INFO(this->get_logger(),
      "装填モーター: MD%d (serial_tx_%d の data[%d]) 出力=%d",
      md_slot_, OUTPUT_DEVICE_ID, md_slot_, CHARGE_MOTOR_POWER);

    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      "joy", 10,
      std::bind(&ChargeNode::joy_callback, this, std::placeholders::_1));

    publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
      "serial_tx_" + std::to_string(OUTPUT_DEVICE_ID), 10);

    mode_pub_ = this->create_publisher<std_msgs::msg::String>("/charge/mode", 10);

    // 状態表示LEDの色 (natsu_ir/ir_led_policy が publish)
    led_sub_ = this->create_subscription<std_msgs::msg::Int16>(
      "ir/led_color", 10,
      std::bind(&ChargeNode::led_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(PUBLISH_RATE_MS),
      std::bind(&ChargeNode::timer_callback, this));
  }

private:
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {

    bool OPTION= msg->buttons[9];
    bool R1 = msg->buttons[5];
    static bool last_option = false;
    static int option_count = 0;
    

    if (OPTION && !last_option)
    {
      option_count++;
    }
    
    if(option_count % 2 == 0)
    {
      data_[md_slot_] = 0; // OPTIONが偶数回押された場合、モーターを止める
    }
    else
    {
      data_[md_slot_] = CHARGE_MOTOR_POWER; // OPTIONが奇数回押された場合、モーターを回す
    }

    if(R1)
    {
        data_[md_slot_] = 0; // R1が押された場合、モーターを止める
    }


    RCLCPP_INFO(this->get_logger(), "MD%d : %d", md_slot_, data_[md_slot_]);

    last_option = OPTION;
  }

  void led_callback(const std_msgs::msg::Int16::SharedPtr msg)
  {

    // LED4本とも同じ色
    for (int i = 0; i < 4; i++) data_[LED_SLOT + i] = msg->data;
  }

  void timer_callback()
  {
    std_msgs::msg::Int16MultiArray msg;
    msg.data = data_;
    publisher_->publish(msg);

    std_msgs::msg::String mode_msg;
    mode_msg.data = "CHARGE";
    mode_pub_->publish(mode_msg);
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr led_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<int16_t> data_;
  // 装填モーターを出すスロット(=MDの番号)。md_number パラメータで決まり、以後変わらない。
  int md_slot_ = DEFAULT_MD_NUMBER;
  bool last_option_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ChargeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}