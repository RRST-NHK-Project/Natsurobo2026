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
#include <std_msgs/msg/float64.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>

// 以下マイコンに合わせて設定
#define OUTPUT_DEVICE_ID 0x03 // 出力マイコン（モーター制御）のID
#define INPUT_DEVICE_ID 0x03 // 入力マイコン（マイクロスイッチやエンコーダ）のID
#define TX16NUM 24            // 送信データ数
#define RX16NUM 17            // 受信データ数

#define PUBLISH_RATE_MS 20 // publish周期(ms), 短くしすぎるとマイコンが処理しきれなくなるので注意

//使用するモーターの選択
#define MODE_MABUCHI
//#define MODE_BLDC

// スティックのデッドゾーン
#define DEADZONE_L 0.3
#define DEADZONE_R 0.3

#define drive_mode (mode_count % 2 == 0)  
#define get_eel_mode (mode_count % 2 == 1)

#if defined(MODE_BLDC)
#define CMD_TOPIC "/odrv_a/axis0/velocity_cmd"
#endif

const int servo_init_deg = 0; //サーボ初期角度（仮）
const int servo_move_deg = 90; //サーボ展開角度（仮）
const int motor_pow = 70; //当然仮の値

// 現在の操作モード(L1で切替)。GUI表示用に /manual/mode へ publish する。
// 偶数=Drive(走行), 奇数=Get_eel(捕獲)。ps4コールバックとpublishタイマで共有するため atomic。
std::atomic<int> g_mode_count{0};

// =================================================================
// HardWareControlノード: ID=3のESP32へモーター指令を送信する
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
        | data[1] | MD1 | -255 ~ 255 |
        | data[2] | MD2 | -255 ~ 255 |
        | data[3] | MD3 | -255 ~ 255 |
        | data[4] | MD4 | -255 ~ 255 |
        | data[5] | MD5 | -255 ~ 255 |
        | data[6] | MD6 | -255 ~ 255 |
        | data[7] | MD7 | -255 ~ 255 |
        | data[8] | MD8 | -255 ~ 255 |
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

        // 現在の操作モードをGUIへ通知(Drive/Get_eel)。定常publishでGUIが常に最新値を持てる
        mode_pub_ = this->create_publisher<std_msgs::msg::String>("/manual/mode", 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&HardWareControl::publisher_timer_callback, this));

        #if defined(MODE_BLDC)

        cmd_pub_ = this->create_publisher<std_msgs::msg::Float64>("CMD_TOPIC", 10);

        #elif defined(MODE_MABUCHI)
        #endif

        // sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        //     "serial_rx_" + std::to_string(device_id_),
        //     10,
        //     std::bind(&HardWareControl::sensor_callback,
        //               this,
        //               std::placeholders::_1));

        RCLCPP_INFO(get_logger(),
                    "HardWareControl: serial_tx_%d 送信開始", OUTPUT_DEVICE_ID);

      //以下追加
        cllect_start_sub_1 = this->create_subscription<std_msgs::msg::Bool>(
          "/collect/start", 10,
          std::bind(&HardWareControl::collect_start_callback, this,
                    std::placeholders::_1));

        cllect_start_sub_2 = this->create_subscription<std_msgs::msg::Bool>(
          "/collect/abort", 10,
          std::bind(&HardWareControl::collect_abort_callback, this,
                    std::placeholders::_1));

        cllect_start_sub_3 = this->create_subscription<std_msgs::msg::Bool>(
          "/collect/done", 10,
          std::bind(&HardWareControl::collect_done_callback, this,
                    std::placeholders::_1));
      //ここまで

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
        bool CIRCLE = msg->buttons[1];
        bool TRIANGLE = msg->buttons[2];
        bool SQUARE = msg->buttons[3];

        bool LEFT = msg->axes[6] == 1.0;
        bool RIGHT = msg->axes[6] == -1.0;
        bool UP = msg->axes[7] == 1.0;
        bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        //bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];
        // static bool last_option = false;
        // static bool option_latch = false;
        static bool last_SQUARE = false; // SQUAREの前回状態を保持する変数
        static bool last_CIRCLE = false; // CIRCLEの前回状態を保持する変数
        static bool last_TRIANGLE = false; // TRIANGLEの前回状態を保持する変数
        static bool last_CROSS = false; // CROSSの前回状態を保持する変数
        static int triangle_count = 0; // TRIANGLEの押下回数をカウントする変数
        static int cross_count = 0; // CROSSの押下回数をカウントする変数

        static bool last_L1 = false; // L1の前回状態を保持する変数
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


        
        static int mode_count = 0; // モード切替のカウンター
        if(L1 && !last_L1) // L1が押された瞬間にモード切替
        {
            mode_count++;
        }
        g_mode_count.store(mode_count); // GUI表示用にモードを共有(/manual/mode)



        //data_[2] = reverserolling ? -20 : 0; // 逆回転モードが有効な場合、回転方向を反転

        if(drive_mode)
        { // ドライブモード時の処理（捕獲モードと間違えて書かないこと）
            static int crosskey_count = 0; // CROSSの押下回数をカウントする変数
            RCLCPP_INFO(this->get_logger(), 
                                 "Now, you are on Mode:Drive.");

            
            // =================================================================
            // CROSS:
            // =================================================================

            // =================================================================
            // CIRCLE: 昇降機構で使用×
            // =================================================================

            // =================================================================
            // TRIANGLE:　「小鰻射出機構」（ブラシレスモーター使用？）
            static int injection_speed = -150; // 射出速度(おそらく上のMDの値の範囲間違ってる。普通に-255~255で制御),実際に試してみると全部負の値で射出できた

            if (TRIANGLE)
            {
                data_[1] = injection_speed;
                data_[2] = injection_speed;
                data_[3] = injection_speed; // 射出部分　出力は一旦50にしておく　要調整
            }
            else
            {
                data_[1] = 0;
                data_[2] = 0;
                data_[3] = 0; // 射出部分　出力は一旦50にしておく　要調整
            }
            RCLCPP_INFO(this->get_logger(),
                                    "motor[1,2,3]: %d,%d,%d", data_[1], data_[2], data_[3]); 
            // =================================================================

            // =================================================================
            // SQUARE:
            // =================================================================

            // =================================================================
            // UP,DOWN:「ピッチ軸回転」
            static int pitch_state= 270 ; // ピッチ軸(縦回転)の角度
            if (DOWN)
            {
                pitch_state = pitch_state + 3; 

                if (pitch_state >270)
                {
                    pitch_state =270; // 上限角度は要調整
                }
            }

            else if (UP)
            {
                pitch_state = pitch_state - 3;

                if (pitch_state < 120)
                {
                    pitch_state = 120; // 下限角度は要調整
                }
            }

            data_[10] = pitch_state;
            
            // =================================================================

            // =================================================================
            // LEFT,RIGHT:「ヨー軸回転」
                static int yaw_state=135; // ヨー軸(横回転)の角度
            if (RIGHT)
            {
                yaw_state = yaw_state + 3; 

                if (yaw_state >270)
                {
                    yaw_state =270; // 上限角度は要調整
                }
            }

            else if (LEFT)
            {
                yaw_state = yaw_state - 3; 

                if (yaw_state < 0)
                {
                    yaw_state = 0; // 下限角度は要調整
                }
            }

            data_[9] = yaw_state; // ヨー軸の角度を配列に格納
        

            RCLCPP_INFO(this->get_logger(), 
        "triangle: %d cross: %d ",triangle_count, crosskey_count); // サーボの角度を表示
            // =================================================================
        } 
        else if (get_eel_mode)
        {
            RCLCPP_INFO(this->get_logger(), 
                                 "Now, you are on Mode:Get_eel.");
            // 捕獲モードの処理をここに記述
                if(TRIANGLE && !last_TRIANGLE)
                {
                    triangle_count++;
                }
                if(triangle_count % 2 == 1)
                {
                    data_[18] = 1; // TRIANGLEが偶数回押された場合、TR2を0に設定
                }
                else if(triangle_count % 2 == 0)
                {
                    data_[18] = 0; // TRIANGLEが奇数回押された場合、TR2を1に設定
                }

                if(CROSS && !last_CROSS)
                {
                    cross_count++;
                }
                if(cross_count % 2 == 1)
                {
                    data_[19] = 1; // CROSSが偶数回押された場合、TR3を0に設定
                }
                else if(cross_count % 2 == 0)
                {
                    data_[19] = 0; // CROSSが奇数回押された場合、TR3を1に設定
                }

                if(SQUARE && last_SQUARE){
                    if(SQUARE_count % 2== 0){
                        data_[11] = servo_init_deg;
                    }
                    else if(SQUARE_count % 2 == 1){
                        data_[11] = servo_move_deg;
                    }
                    SQUARE_count++;
                    }
                    
                    

                if(CIRCLE && !last_CIRCLE){
                    if(CIRCLE_count % 2 == 0){
                        data_[17] = 1;
                    }
                    else if(CIRCLE_count % 2 == 1){
                        data_[17] = 0;
                    }
                    CIRCLE_count++;
                
                }
                if(UP){
                    data_[4] = -motor_pow; // 上昇
                }
                if(DOWN){
                    data_[4] = motor_pow; // 下降
                }
            }

        RCLCPP_INFO(this->get_logger(), 
        "data_[9,10,16]:, %d, %d, %d, data_[13,14,15]: %d, %d, %d, data_[4]: %d, data_[11]: %d,data_[17]: %d]", data_[9], data_[10], data_[16], data_[13], data_[14], data_[15], data_[4], data_[11], data_[17]); // 装填機構のモーターの速度とハンドアームのワークを掴む機構の開閉を表示
    
    last_L1 = L1; // L1の状態を更新
    last_SQUARE = SQUARE; // SQUAREの状態を更新
    last_CIRCLE = CIRCLE; // CIRCLEの状態を更新
    last_TRIANGLE = TRIANGLE; // TRIANGLEの状態を更新
    last_CROSS = CROSS; // CROSSの状態を更新
        // 配列操作ここまで
    }


    // 自動回収フラグの受信（/collect/start, /collect/abort, /collect/done）
    void collect_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            auto_collect_active_ = true;
            auto_collect_abort_ = false;
            RCLCPP_INFO(this->get_logger(), "自動回収を開始します (/collect/start)");
        }        
    }

    void collect_abort_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            auto_collect_abort_ = true;
            auto_collect_active_ = false;
            RCLCPP_WARN(this->get_logger(), "自動回収を中断しました (/collect/abort)");
        }
    }

    void collect_done_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data)
        {
            auto_collect_active_ = false;
            RCLCPP_INFO(this->get_logger(), "自動回収が完了しました (/collect/done)");
        }
    }

    // TODO: 自動回収の動作シーケンスを実装する（現状はビルドを通すための空実装）
    void run_auto_collect()
    {
    }

    // publish
    void publisher_timer_callback()
    {
        std_msgs::msg::Int16MultiArray msg;

        //以下追加
        if (auto_collect_active_){
            run_auto_collect();
           }
        //ここまで


        msg.data = data_;

        publisher_->publish(msg);

        // 現在モードをGUIへ publish (偶数=Drive, 奇数=Get_eel)
        std_msgs::msg::String mode_msg;
        mode_msg.data = (g_mode_count.load() % 2 == 0) ? "DRIVE" : "GET_EEL";
        mode_pub_->publish(mode_msg);

        #if defined(MODE_BLDC)
        std_msgs::msg::Float64 cmd_msg;
        cmd_msg.data = target_vel_;
        cmd_pub_->publish(cmd_msg);
        #endif
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

    // int16_t ENC1 = msg->data[1];//ENC1~3はsummer2026_odometry.cppで貰い受けた。
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_; // /manual/mode (GUI表示用)
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cllect_start_sub_1;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cllect_start_sub_2;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr cllect_start_sub_3;

    bool auto_collect_active_ = false;
    bool auto_collect_abort_ = false;

    int SQUARE_count = 0; 
    int CIRCLE_count = 0;

    #if defined(MODE_BLDC)
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr cmd_pub_;
    double target_vel_;// ブラシレスモーターの速度指令値用の変数
    #endif

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

    // ID=3: モーター出力ノード
    auto hardware_control = std::make_shared<HardWareControl>();
    exec.add_node(hardware_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}
#if(defined(MODE_MABUCHI) + defined(MODE_BLDC)) != 1
#error "Please select ONE motor"
#endif