/*
naturobo機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// まだ未確認なので絶対に許可なしに起動しないこと！！
// 破壊しても自己責任！！

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

// 自作
#include "summer2026_odometry.hpp"

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
// マイクロスイッチの状態（ID=3のESP32から受信、2ノード間で共有）
// atomic: スレッドセーフに読み書きするため
std::atomic<int16_t> g_micro1_sw{0}; // マイクロスイッチ(上): 1=押されている
std::atomic<int16_t> g_micro2_sw{0}; // マイクロスイッチ(下): 1=押されている
std::atomic<int16_t> g_micro3_sw{0}; // マイクロスイッチ(外側): SW3
std::atomic<int16_t> g_micro4_sw{0}; // マイクロスイッチ(内側): SW4
std::atomic<int16_t> g_enc1_val{0};  // エンコーダ1: data[1]から受信

// フォークリフト座標管理 (EncoderCoordinator)
// エンコーダ減少 -> 座標増加 / エンコーダ増加 -> 座標減少
std::atomic<int32_t> g_rotation_count{0};     // エンコーダの回転数(巻回り数)
std::atomic<int64_t> g_zero_offset{0};        // 下端リセット時の絶対エンコーダ値
std::atomic<int16_t> g_last_enc1_val{0};      // 前回のエンコーダ生値
std::atomic<bool> g_coord_initialized{false}; // 初期化フラグ
std::atomic<int64_t> g_abs_coord{0};          // 最終的な高さ座標(下端=0方向=プラス)

// =================================================================

// =================================================================
// SwitchInputノード: ID=3のESP32からマイクロスイッチの状態を受信する
// =================================================================
class SwitchInput : public rclcpp::Node
{
public:
    SwitchInput()
        : Node("switch_input_" + std::to_string(INPUT_DEVICE_ID))
    {

        sw_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(INPUT_DEVICE_ID),
            10,
            std::bind(&SwitchInput::sw_callback, this, std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "SwitchInput: serial_rx_%d を受信開始", INPUT_DEVICE_ID);
    }

private:
    void sw_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg)
    {
        // SW4 (data[12]) まで使用するため、サイズチェックを13以上に変更
        if (msg->data.size() < 13)
        {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                                 "serial_rx_%d: データが短すぎます (%zu)",
                                 INPUT_DEVICE_ID, msg->data.size());
            return;
        }

        // マイクロスイッチの値を更新
        // マイクロスイッチの値を更新 (物理配線に合わせて修正: 9=下, 10=上)
        g_micro1_sw = msg->data[10]; // 上端スイッチ(micro1)
        g_micro2_sw = msg->data[9];  // 下端スイッチ(micro2)
        g_micro3_sw = msg->data[11];
        g_micro4_sw = msg->data[12];
        g_enc1_val = msg->data[1];

        // =============================================================
        // 座標調査・ラップアラウンド計算実装
        // =============================================================
        int16_t current_enc1 = msg->data[1];

        if (!g_coord_initialized.load())
        {
            g_last_enc1_val.store(current_enc1);
            g_coord_initialized.store(true);
        }

        // =====================================================================
        // 【重要】エンコーダの「飛躍（16bitハードの限界）」は 32768 または 65536
        // （「1周=8000」は機械的な回転数であり、デジタル的なラップアラウンド値とは別）
        // =====================================================================
        const int HALF_ENCODER = 16384;    // デジタルデータの飛躍値の半分
        const int64_t ENCODER_MAX = 32768; // デジタルデータの飛躍幅

        int diff = (int)current_enc1 - (int)g_last_enc1_val.load();
        int32_t r_count = g_rotation_count.load();

        if (diff > HALF_ENCODER)
        {
            r_count--;
        }
        else if (diff < -HALF_ENCODER)
        {
            r_count++;
        }

        g_rotation_count.store(r_count);
        g_last_enc1_val.store(current_enc1);

        // 連続化された総エンコーダカウント
        int64_t total_encoder = (int64_t)r_count * ENCODER_MAX + (int64_t)current_enc1;

        // 下端スイッチ(data[9])で座標リセット用のオフセットを設定
        if (msg->data[9] != 0)
        {
            g_zero_offset.store(total_encoder);
            RCLCPP_INFO(get_logger(), "[COORD RESET!] 下端ボタン押下により座標0へオフセット設定");
        }

        // 最終的な高さを計算
        int64_t zero_offset = g_zero_offset.load();
        int64_t abs_coord = -(total_encoder - zero_offset);
        g_abs_coord.store(abs_coord);

        // ★ここで 8000 で割ることで「物理的な1回転」を算出します
        double rot = (double)abs_coord / 8192.0;

        // 以下リアルタイムで数値取るデバックログ　重いとき消すこと推奨
        if (diff != 0)
        {
            // RCLCPP_INFO(get_logger(),
            //             "\n--- ROTATION DEBUG ---\n"
            //             "  生値の変化 : %d -> %d (diff: %d)\n"
            //             "  デジタルラップ : %d 回\n"
            //             "  絶対カウント   : %ld\n"
            //             "  現在回転数     : %.3f 回転 (1周8000)\n"
            //             "----------------------",
            //             (int)current_enc1 - diff, (int)current_enc1, diff,
            //             (int)r_count, abs_coord, rot);
        }

        // --- 通信デバッグ追加 ---
        static uint64_t packet_count = 0;
        packet_count++;

        // 状態変化時のみ即時表示
        static int16_t l9 = 0, l10 = 0, l11 = 0, l12 = 0;
        if (msg->data[9] != l9 || msg->data[10] != l10 || msg->data[11] != l11 || msg->data[12] != l12)
        {
            // RCLCPP_INFO(get_logger(), "SW Changed! [下(9):%d, 上(10):%d, 外(11):%d, 内(12):%d]",
            //             msg->data[9], msg->data[10], msg->data[11], msg->data[12]);
            l9 = msg->data[9];
            l10 = msg->data[10];
            l11 = msg->data[11];
            l12 = msg->data[12];
        }

        // 定期ダンプに受信件数を追加
        // RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        //                      "RX Heartbeat (Total:%lu) | Dump: [%d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d, %d]",
        //                      packet_count,
        //                      msg->data[0], msg->data[1], msg->data[2], msg->data[3],
        //                      msg->data[4], msg->data[5], msg->data[6], msg->data[7],
        //                      msg->data[8], msg->data[9], msg->data[10], msg->data[11],
        //                      msg->data[12], msg->data[13], msg->data[14], msg->data[15]);

        // SW3, SW4専用の明示的なデバッグ
        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
                             "【通信確認】SW3(外側):%d, SW4(内側):%d", msg->data[11], msg->data[12]);
    }

    rclcpp::Subscription<std_msgs::msg::Int16MultiArray>::SharedPtr sw_sub_;
};

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

        bool CROSS = msg->buttons[0];
        // bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

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
        // CROSS:「ハンド操作」（サーボ何個使うかわからないので処理未記入）
            static int cross_state = 0; 
            
        if (CROSS && cross_state == 0){
        
        }
        else if (!CROSS && cross_state == 1){

        }
        
        if (CROSS) {
            cross_state = 1;
        }

        // =================================================================

    
        // =================================================================
        // CIRCLE: 足回りで使用×
        // =================================================================
 

        // =================================================================
        // TRIANGLE:　「小鰻射出機構」（ブラシレスモーター使用？）
            if (TRIANGLE){
                data_[1] = 50; // 射出部分　出力は一旦50にしておく　要調整
            }
        // =================================================================


        // =================================================================
        // SQUARE:　「ハンド回転」
            static int square_state = 0;
            if (SQUARE && square_state == 0){
                data_[9] = 0; // 角度は要調整
            }
            else if (!SQUARE && square_state == 1){
                data_[9] = 90; // 角度は要調整
            }
             square_state = SQUARE;
        // =================================================================


        // =================================================================
        // UP,DOWN:「昇降機構」
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

        // ★★★ コントローラーの操作が無い時でも、マイクロスイッチの安全停止を最優先で適用する ★★★
        // （PS4コントローラーのイベントが来ない間も常に制限をかけるため、ここに記述する）
        int16_t micro1_sw = g_micro1_sw.load();
        int16_t micro2_sw = g_micro2_sw.load();

        // 上昇中（data_[2] が正の値）かつ 上端スイッチが押されている場合
        if (micro2_sw == 1 && data_[2] > 0)
        {
            data_[2] = 0;
            // 重い場合以下のデバックのコメントアウト可
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】上端リミット到達！モーターの上昇を即時遮断しました！");
        }

        // 下降中（data_[2] が負の値）かつ 下端スイッチが押されている場合
        if (micro1_sw == 1 && data_[2] < 0)
        {
            data_[2] = 0;
            // 重い場合以下のデバックのコメントアウト可
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 500, "【安全装置】下端リミット到達！モーターの下降を即時遮断しました！");
        }

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

    //オドメトリノード
    auto odometry = std::make_shared<Shivalian_control>(RX_DEVICE_ID);
    exec.add_node(odometry);

    exec.spin();

    rclcpp::shutdown();
    return 0;
}