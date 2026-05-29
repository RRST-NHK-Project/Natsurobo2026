/*
夏ロボ機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// これから発展予定　データの割当はまだ調べてないのでのちのち変更予定 一旦足回りの方に合わせておく


#include <atomic>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>
#include <cstdint>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 2 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID 3  // 入力マイコン（マイクロスイッチやエンコーダ）のID
#define TX16NUM 24         // 送信データ数
#define RX16NUM 17         // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3



// =================================================================
// HardWareControlノード: ID=2のESP32へモーター指令を送信する
// =================================================================
class HardWareControl : public rclcpp::Node
{
public:
    HardWareControl()
        : Node("hardware_control_" + std::to_string(OUTPUT_DEVICE_ID))
    {

        // 配列を0で初期化
        data_.assign(TX16NUM, 0);
        /*
        マイコンに送信される配列"data_"
        debug: 機能未割り当て, MD: モータードライバー, TR: トランジスタ
        | data[n] | 詳細 | 範囲 |
        | ---- | ---- | ---- |
        | data[0] | debug | 0 or 1 |
        | data[1] | MD1 | -100 ~ 100 |
        | data[2] | MD2 | -100 ~ 100 |
        | data[3] | MD3 | -100 ~ 100 |
        | data[4] | MD4 | -100 ~ 100 |
        | data[5] | MD5 | -100 ~ 100 |
        | data[6] | MD6 | -100 ~ 100 |
        | data[7] | MD7 | -100 ~ 100 |
        | data[8] | MD8 | -100 ~ 100 |
        | data[9] | Servo1 | 0 ~ 270 |
        | data[10] | Servo2 | 0 ~ 270 |
        | data[11] | Servo3 | 0 ~ 270 |
        | data[12] | Servo4 | 0 ~ 270 |
        | data[13] | Servo5 | 0 ~ 270 |
        | data[14] | Servo6 | 0 ~ 270 |
        | data[15] | Servo7 | 0 ~ 270 |
        | data[16] | Servo8 | 0 ~ 270 |
        | data[17] | TR1 | 0 or 1|
        | data[18] | TR2 | 0 or 1|
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        // joyノードのSubscribe
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10,
            std::bind(&HardWareControl::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(OUTPUT_DEVICE_ID), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        // sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     "serial_rx_" + std::to_string(device_id_),
        //     10,
        //     std::bind(&HardWareControl::sensor_callback,
        //               this,
        //               std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "HardWareControl: serial_tx_%d 送信開始", OUTPUT_DEVICE_ID);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        // bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
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

        // マイクロスイッチの状態をグローバル変数から取得
        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();
        int16_t micro3_sw = g_micro3_sw.load();
        int16_t micro4_sw = g_micro4_sw.load();

        // 制御ノード側のデバッグログ
        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
        //                      "【制御ノード表示】SW状態: 上=%d (%s), 下=%d (%s), 外=%d (%s), 内=%d (%s)",
        //                      micro1_sw, micro1_sw ? "停止" : "通常",
        //                      micro2_sw, micro2_sw ? "停止" : "通常",
        //                      micro3_sw, micro3_sw ? "停止" : "通常",
        //                      micro4_sw, micro4_sw ? "停止" : "通常");

        // 以降、配列data_を操作する
        // ボタン設定は適当に借り決め　必要に応じて変更予定

        // =================================================================
        // CROSS:「ハンド操作」
        // =================================================================

    
        // =================================================================
        // CIRCLE: 「ハンド回転」
        // =================================================================
 

        // =================================================================
        // TRIANGLE:　「小鰻射出機構」（ブラシレスモーター使用？）
        // =================================================================


        // =================================================================
        // SQUARE:　「昇降機構」
        // =================================================================


        // =================================================================
        // UP,DOWN:
        // =================================================================


        // =================================================================
        // LEFT,RIGHT:
        // =================================================================




        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        msg.data = data_;

        publisher_->publish(msg);
    }

    // void
    // sensor_callback(
    //     const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
    //     // 最低限：サイズチェック
    //     if (msg->data.size() < RX16NUM) {
    //         RCLCPP_WARN(this->get_logger(),
    //                     "serial_rx_%d: data too short (%zu)",
    //                     device_id_, msg->data.size());
    //         return;
    //     }

    // int16_t ENC1 = msg->data[1];
    // int16_t ENC2 = msg->data[2];
    // int16_t ENC3 = msg->data[3];
    // int16_t ENC4 = msg->data[4];
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

    // 以降、受信データを使った処理を記述
    // エンコーダースイッチの状態を保存（モーター制御で使用）
    // micro1_sw_ = SW1;
    // micro2_sw_ = SW2;

    // デバッグ: マイクロスイッチの受信値を確認
    // RCLCPP_INFO(get_logger(),
    //             "[マイクロSW] 上(L1禁止用)=%s  下(R1禁止用)=%s",
    //             micro1_sw_ ? "★押されている" : "　押されていない",
    //             micro2_sw_ ? "★押されている" : "　押されていない");

    // 受信データ処理ここまで
    // }

    uint8_t device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet R1 Motion Ctrl";
    int result = std::system(figletout.c_str());
    if (result != 0)
    {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    // ID=2: モーター出力ノード
    auto hardware_control = std::make_shared<HardWareControl>();
    exec.add_node(hardware_control);

    // ID=3: マイクロスイッチ＆エンコーダ入力ノード
    auto switch_input = std::make_shared<SwitchInput>();
    exec.add_node(switch_input);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}