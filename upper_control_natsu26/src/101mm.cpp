/*
Serial_Bridgeノードのホスト側プログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <vector>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include <std_msgs/msg/int16_multi_array.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>

// 以下マイコンに合わせて設定
#define TX_DEVICE_ID 3 // 送信先マイコンのID

#define TX16NUM 24 // 送信データ数
#define RX16NUM 17 // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

#define drive_mode (!(L1 && last_L1))   // L1を押していないときはドライブモード
#define get_eel_mode (L1 && !last_L1) // L1を押しているときは捕獲モード

class unaginobori2026 : public rclcpp::Node {
public:
    unaginobori2026(uint8_t tx_device_id)
        : Node("unaginobori2026"),
          tx_device_id_(tx_device_id) {

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
            std::bind(&unaginobori2026::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&unaginobori2026::publisher_timer_callback, this));

        //sensor_callbackは廃止しました。
        
        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
    }

private:
    void ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {

        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        // float LS_X = -1 * msg->axes[0];
        // float LS_Y = msg->axes[1];
        // float RS_X = -1 * msg->axes[3];
        // float RS_Y = msg->axes[4];

        // bool CROSS = msg->buttons[0];
        bool CIRCLE = msg->buttons[1];
        // bool TRIANGLE = msg->buttons[2];
        // bool SQUARE = msg->buttons[3];

        // bool LEFT = msg->axes[6] == 1.0;
        // bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        bool R3 = msg->buttons[12];

        // static bool last_option = false;
        // static bool option_latch = false;

        // static bool last_share = false;
        // static bool share_latch = false;
        bool last_CIRCLE = false;
        bool last_L1 = false;
        // bool last_R1 = false;

        static int count = 0;

        // 以降、配列data_を操作する
        if(drive_mode) {
            // ドライブモードの処理

        if (CIRCLE && !last_CIRCLE)
            {   // CIRCLEが押されたときに一度だけ実行される処理（CIRCLEを押すたびに段差超え処理を進める）
            // 自動化出来るか分からんから一応完全マニュアル操作を想定

            if (count % 3 == 0)
            {
                data_[17] = 1; // data_[17]~data_[24]までのどっか(前輪) = 1;//4輪をエアシリンダで持ち上げる
                // data_[17]~data_[24]までのどっか(後輪) = 1;
                count++;
            }
            else if (count % 3 == 1)
            {
                data_[18] = 1;
                data_[17] = 0; // data_[17]~data_[24]までのどっか(前輪) = 0;//前輪格納（手動で前進してね^^）
                count++;
            }
            else
            {
                // data_[17]~data_[24]までのどっか(後輪) = 0;//後輪格納;
                data_[18] = 0;
                count++;
            }

            }

        /*if(R1 && !last_R1){//段差超えの逆操作（一応実装しておく）
            if(count % 3 == 0){
                data_[17] = 0; // data_[17]~data_[24]までのどっか(前輪) = 0;//
                // data_[17]~data_[24]までのどっか(後輪) = 0;
                count--;
            }
            else if(count % 3 == 1){
                data_[18] = 0;
                data_[17] = 1; // data_[17]~data_[24]までのどっか(前輪) = 1;
                count--;
            }
            else{
                // data_[17]~data_[24]までのどっか(後輪) = 1;
                data_[18] = 1;
                count--;
            }
        }*/
        //last_R1 = R1;
        if(R3) {
            //前進用ホイールのモータの番号data_[1~4] = 15; 
            //前進用ホイールのモータの番号data_[1~4] = 15; 
        }
        /*if(DOWN) {
            //前進用ホイールのモータの番号data_[1~4] = -15;
            //前進用ホイールのモータの番号data_[1~4] = -15;
        }*/

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);
        }
        else if(get_eel_mode) {
            // 捕獲モードの処理
            /*e.g.
            if(CIRCLE)
            if(CROSS)*/
        

        }

        last_CIRCLE = CIRCLE;
        last_L1 = L1;

        // 配列操作ここまで
    }

    // publish
    void publisher_timer_callback() {
        std_msgs::msg::Int16MultiArray msg;

        //一応...
        RCLCPP_INFO(
            get_logger(),
            "data_[22]=%d, data_[23]=%d",//"Front=%d, Back=%d",//確定したら数字を入れる
            data_[22], data_[23]);//data_[1~4], data_[1~4]);

        msg.data = data_;

        publisher_->publish(msg);
    }

    uint8_t tx_device_id_;
    uint8_t rx_device_id_;

    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<int16_t> data_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet Air 2026";
    int result = std::system(figletout.c_str());
    if (result != 0) {
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
        std::cerr << "Please install 'figlet' with the following command:"
                  << std::endl;
        std::cerr << "sudo apt install figlet" << std::endl;
        std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                  << std::endl;
    }

    rclcpp::executors::MultiThreadedExecutor exec;

    auto upper_control = std::make_shared<unaginobori2026>(TX_DEVICE_ID);
    exec.add_node(upper_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
};