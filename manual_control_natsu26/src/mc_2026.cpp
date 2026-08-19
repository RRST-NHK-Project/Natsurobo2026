/*
naturobo機構制御
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

// まだ未確認なので絶対に許可なしに起動しないこと！！
// 破壊しても自己責任！！

#include <algorithm>
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
#include <std_msgs/msg/int32.hpp>
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

#define drive_mode (mode_count % 3 == 0)
#define get_eel_mode (mode_count % 3 == 1)
#define shoot_mode (mode_count % 3 == 2)

#if defined(MODE_BLDC)
#define CMD_TOPIC "/odrv_a/axis0/velocity_cmd"
#endif

const int servo_init_deg = 0; //サーボ初期角度（仮）
const int servo_move_deg = servo_init_deg + 170; //サーボ展開角度（仮）
const int motor_pow = 70; //当然仮の値
const int motor_pow2 = 50; //当然仮の値

// 現在の操作モード(SHAREで切替)。GUI表示用に /manual/mode へ publish する。
// mode_count%3 で 0=Drive(走行) -> 1=Get_eel(捕獲) -> 2=Shoot(射出) と巡回。
// ps4コールバックとpublishタイマで共有するため atomic。
std::atomic<int> g_mode_count{0};

// =================================================================
// SHOOTモード用: カゴ照準プリセット(仮値)
//   3x3グリッド(H字型, 欠番=[1][0],[1][2])の各セルに yaw/pitch を割り当てる。
//   参照は [x][y](カーソル shoot_x_/shoot_y_ とそのままの順)。欠番セルの値は未使用。
//   x=横方向(ヨー), y=縦方向(ピッチ)。ソース上は 1行=1つのx列。
//            y=0    y=1    y=2
//     x=0  (0,0)  (0,1)  (0,2)
//     x=1  [欠]   (1,1)  [欠]
//     x=2  (2,0)  (2,1)  (2,2)   ← 起動時のカーソルは [0][2]
// =================================================================
// ヨー角
static const int kYawPreset[3][3] = {
    { 45,  90, 100}, // x=0列
    {  0, 135,   0}, // x=1列: [1][0],[1][2]は欠番
    {225, 180, 200}, // x=2列: [0][2]が初期位置
};

// ピッチ角
static const int kPitchPreset[3][3] = {
    {200, 160, 120}, // x=0列
    {  0, 160,   0}, // x=1列: [1][0],[1][2]は欠番
    {200, 160, 120}, // x=2列: [0][2]が初期位置
};

// 射出速度
static const int kInjectionSpeedPreset[3][3] = {
    {-130, -150, -180}, // x=0列: [0][0]=G0(手前左), [0][1]=B0(中央左), [0][2]=B4(奥左)
    {-150, -150, -150}, // x=1列: [1][1]=B1(中央)のみ有効。[1][0],[1][2]は欠番(既定値)
    {-130, -150, -180}, // x=2列: [2][0]=B3(手前右), [2][1]=B2(中央右), [2][2]=G1(奥右)
};

// GUIカゴ対応テーブル (0,1が緑カゴ、2〜6が青カゴ)
static const int kBasketCellXy[7][2] = {
    {0, 0}, // G0 (手前左・緑)
    {2, 2}, // G1 (奥右・緑)
    {0, 1}, // B0 (中央左・青)
    {1, 1}, // B1 (中央・青)
    {2, 1}, // B2 (中央右・青)
    {2, 0}, // B3 (手前右・青)
    {0, 2}, // B4 (奥左・青)
};

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

        // GUI(コンソール)からのカゴ直接指定。セル番号(0〜6)を受け取り、SHOOT中のみ
        // 該当セルのyaw/pitchをサーボへ反映する。dpadカーソルと同じ shoot_x_/shoot_y_ を動かす。
        basket_sub_ = this->create_subscription<std_msgs::msg::Int32>(
          "/manual/basket", 10,
          std::bind(&HardWareControl::BasketCallback, this,
                    std::placeholders::_1));

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
        bool R1 = msg->buttons[5];

        // float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        //bool R2 = msg->buttons[7];

        bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];
        // static bool last_option = false;
        // static bool option_latch = false;
        static bool last_CROSS = false; // CROSSの前回状態を保持する変数
        static int CROSS_count = 0; // CROSSの押下回数をカウントする変数
        static bool last_SQUARE = false; // SQUAREの前回状態を保持する変数
        static bool last_CIRCLE = false; // CIRCLEの前回状態を保持する変数
        static bool last_TRIANGLE = false; // TRIANGLEの前回状態を保持する変数
        static int triangle_count = 0; // TRIANGLEの押下回数をカウントする変数
        static int last_L1 = false; // L1の前回状態を保持する変数
        static int L1_count = 0; 

        static bool last_SHARE = false; // SHAREの前回状態を保持する変数
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


        static bool topic_received = false;

        if(!topic_received){
            data_[11] = servo_init_deg;
            topic_received = true;
        }

        // 捕獲モードと射出モードで共通の「ホッパー + リロード + 射出」操作。
        // 両モードで同じボタン・同じカウンタを使うので、モードを跨いでもシリンダーの
        // 開閉状態が食い違わない(射出モードで開けて捕獲モードで閉じる、ができる)。
        auto hopper_and_shoot = [&]() {
            // L1: エアシリンダー(data_[19]) をトグル
            if (L1 && !last_L1) {
                if (L1_count % 2 == 0) { data_[19] = 0; }
                else                   { data_[19] = 1; }
                L1_count++;
            }

            // CROSS: エアシリンダー(data_[17]) をトグル
            if (CROSS && !last_CROSS) {
                if (CROSS_count % 2 == 0) { data_[17] = 0; }
                else                      { data_[17] = 1; }
                CROSS_count++;
            }

            // TRIANGLE: 詰まり防止(リロード)。data_[18] をトグル。
            // L1/CROSSと違い、こちらは押した1回目からONになる(カウントしてから判定するため)。
            if (TRIANGLE && !last_TRIANGLE) {
                triangle_count++;
            }
            if (triangle_count % 2 == 1) {
                data_[18] = 1;
            } else {
                data_[18] = 0;
            }

            // R1: 射出。速度は選択中のカゴ(shoot_x_/shoot_y_)のプリセット値を使う。
            if (R1) {
                const int spd = current_injection_speed();
                data_[1] = spd;
                data_[2] = spd;
                data_[3] = spd;
                RCLCPP_INFO(this->get_logger(),
                    "射出: cell=[%d][%d](x,y) injection_speed=%d",
                    shoot_x_, shoot_y_, spd);
            } else {
                data_[1] = 0;
                data_[2] = 0;
                data_[3] = 0;
            }
        };
        
        static int mode_count = 0; // モード切替のカウンター
        if(SHARE && !last_SHARE) // SHAREが押された瞬間にモード切替
        {
            mode_count++;
            // 射出出力は捕獲/射出モードでしか書かないので、R1を握ったまま走行モードへ
            // 切り替えると回りっぱなしになる。モードが変わった時点で必ず止める。
            data_[1] = 0;
            data_[2] = 0;
            data_[3] = 0;
        }
        g_mode_count.store(mode_count); // GUI表示用にモードを共有(/manual/mode)



        //data_[2] = reverserolling ? -20 : 0; // 逆回転モードが有効な場合、回転方向を反転

        if(drive_mode)
        { // ドライブモード時の処理（捕獲モードと間違えて書かないこと）
            
            RCLCPP_INFO(this->get_logger(), 
                                 "Now, you are on Mode:Drive.");

            
            // =================================================================
            // CROSS:
            // =================================================================

            // =================================================================
            // CIRCLE: 昇降機構で使用×
            // =================================================================

            // =================================================================
            // TRIANGLE:　「小鰻射出機構」（ブラシレスモーター使用？）//<-移動

            RCLCPP_INFO(this->get_logger(),
                                    "motor[1,2,3]: %d,%d,%d", data_[1], data_[2], data_[3]); 
            // =================================================================

            // =================================================================
            // SQUARE:
            // =================================================================

            // =================================================================
            // UP,DOWN:「ピッチ軸回転」
            // 角度はメンバ pitch_deg_ が持つ。射出モードで選んだプリセット位置から
            // そのまま続けて微調整できる(モードを跨いでも値が消えない)。
            if (DOWN)
            {
                pitch_deg_ = pitch_deg_ + 3;

                if (pitch_deg_ > 270)
                {
                    pitch_deg_ = 270; // 上限角度は要調整
                }
            }

            else if (UP)
            {
                pitch_deg_ = pitch_deg_ - 3;

                if (pitch_deg_ < 90)
                {
                    pitch_deg_ = 90; // 下限角度は要調整
                }
            }
            
            // =================================================================

            // =================================================================
            // LEFT,RIGHT:「ヨー軸回転」
            if (RIGHT)
            {
                yaw_deg_ = yaw_deg_ + 3;

                if (yaw_deg_ > 270)
                {
                    yaw_deg_ = 270; // 上限角度は要調整
                }
            }

            else if (LEFT)
            {
                yaw_deg_ = yaw_deg_ - 3;

                if (yaw_deg_ < 0)
                {
                    yaw_deg_ = 0; // 下限角度は要調整
                }
            }

        RCLCPP_INFO(this->get_logger(), 
         "微調整: yaw=%d pitch=%d", yaw_deg_, pitch_deg_); // サーボの角度を表示
        

            // =================================================================
        } 
        else if (get_eel_mode)
        {
            RCLCPP_INFO(this->get_logger(), 
                                 "Now, you are on Mode:Get_eel.");
            // 捕獲モードの処理をここに記述

                // L1/CROSS(エアシリンダー) / TRIANGLE(リロード) / R1(射出)。
                // 射出モードと全く同じ操作なので共通のラムダにまとめてある。
                hopper_and_shoot();

        }
            
        else if (shoot_mode) // シュートモード
        {
            RCLCPP_INFO(this->get_logger(),
                                 "Now, you are on Mode:Shoot.");

            //   参照は [x][y]。
            //          y=0    y=1    y=2
            //   x=0  (0,0)  (0,1)  (0,2)
            //   x=1  [欠]   (1,1)  [欠]
            //   x=2  (2,0)  (2,1)  (2,2)
            //   ※[1][1]は横方向のみ通過可(縦移動は不可)
            //   90度回転した工の字をイメージしてください。
            //   カーソル位置 shoot_x_/shoot_y_ はメンバ変数。dpad(このコールバック)と
            //   GUIカゴ選択(/manual/basket コールバック)の両方から動かせる。

            static bool last_LEFT = false, last_RIGHT = false, last_UP = false, last_DOWN = false;

            // 左右入力(押した瞬間のみ1マス): RIGHT=x+1, LEFT=x-1
            if ((RIGHT && !last_RIGHT) || (LEFT && !last_LEFT)) {
                int nx = std::clamp(shoot_x_ + (RIGHT ? 1 : -1), 0, 2);
                // 移動先が欠番([1][0],[1][2])でなければ確定
                if (!(nx == 1 && (shoot_y_ == 0 || shoot_y_ == 2))) shoot_x_ = nx;
                // 欠番なら動かない(中央行[1][*]は列1も含めて自由)
            }

            // 十字キー上下入力: UP=奥(y+1), DOWN=手前(y-1)
            if ((UP && !last_UP) || (DOWN && !last_DOWN)) {
                if (shoot_y_ == 1) {
                    // 中央行: x=1(中央カゴ)からは上下の欠番へ行けないため、x!=1 のみ縦移動
                    if (shoot_x_ != 1) shoot_y_ = std::clamp(shoot_y_ + (UP ? 1 : -1), 0, 2);
                } else {
                    // y=0(手前) または y=2(奥) からは中央行(y=1)へ戻る
                    shoot_y_ = 1;
                }
            }

            // カゴを選び直した瞬間だけ、そのセルのプリセット角を現在角へ代入する。
            // 毎回代入すると走行モードでやった微調整がここで巻き戻ってしまうため、
            // カーソルが動いた時だけにする(GUI指定 /manual/basket も同じ扱い)。
            if (shoot_x_ != last_cursor_x_ || shoot_y_ != last_cursor_y_) {
                yaw_deg_   = kYawPreset[shoot_x_][shoot_y_];
                pitch_deg_ = kPitchPreset[shoot_x_][shoot_y_];
                last_cursor_x_ = shoot_x_;
                last_cursor_y_ = shoot_y_;
            }

            // ホッパーのエアシリンダー(L1/CROSS)、リロード(TRIANGLE)、射出(R1)。
            // 捕獲モードと同じ割当・同じカウンタなので、モードを跨いでも状態が食い違わない。
            hopper_and_shoot();

            last_LEFT = LEFT;
            last_RIGHT = RIGHT;
            last_UP = UP;
            last_DOWN = DOWN;

            RCLCPP_INFO(this->get_logger(),
                "shoot_mode cursor=[%d][%d](x,y) yaw=%d pitch=%d injection_speed=%d",
                shoot_x_, shoot_y_, yaw_deg_, pitch_deg_, current_injection_speed());
        }

        // 照準サーボへの反映はここ1箇所だけ。モードごとに書いたり書かなかったりすると
        // モード切替のたびに角度が飛ぶので、全モード共通で最後にまとめて入れる。
        data_[9]  = yaw_deg_;
        data_[10] = pitch_deg_;

        RCLCPP_INFO(this->get_logger(), 
        "data[1,2,3]: %d,%d,%d data_[18,19]: %d, %d. data[11]: %d. data[17]: %d. data[4]: %d",data_[1], data_[2], data_[3], data_[18], data_[19], data_[11], data_[17], data_[4]); // 装填機構のモーターの速度とハンドアームのワークを掴む機構の開閉を表示

    
    last_SHARE = SHARE; // SHAREの状態を更新
    last_SQUARE = SQUARE; // SQUAREの状態を更新
    last_CIRCLE = CIRCLE; // CIRCLEの状態を更新
    last_TRIANGLE = TRIANGLE; // TRIANGLEの状態を更新
    last_L1 = L1; // L1の状態を更新
    last_CROSS =CROSS;
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

    // 現在選択中のカゴ(shoot_x_/shoot_y_)に対応する射出速度を返す。
    // 実機では全て負の値で射出できたので、値の範囲は -255 〜 255(MDの指令値)。
    int current_injection_speed() const
    {
        return kInjectionSpeedPreset[shoot_x_][shoot_y_];
    }

    // GUIからのカゴ直接指定(/manual/basket, セル番号0〜6)。
    // SHOOTモード中のみ有効。カーソル(shoot_x_/shoot_y_)を該当セルへ移動し、
    // 角度を data_[9](yaw)/data_[10](pitch) に反映する(次のtimerで送信される)。
    void BasketCallback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        const int idx = msg->data;
        if (idx < 0 || idx >= 7) {
            RCLCPP_WARN(this->get_logger(), "/manual/basket: 範囲外のセル番号 %d (0〜6)", idx);
            return;
        }
        if (g_mode_count.load() % 3 != 2) {
            RCLCPP_WARN(this->get_logger(), "/manual/basket: SHOOTモードでないため無視 (idx=%d)", idx);
            return;
        }
        shoot_x_ = kBasketCellXy[idx][0];
        shoot_y_ = kBasketCellXy[idx][1];
        // 現在角へプリセットを代入し、カーソル位置も同期する(ps4側で二重に代入しないため)。
        yaw_deg_   = kYawPreset[shoot_x_][shoot_y_];
        pitch_deg_ = kPitchPreset[shoot_x_][shoot_y_];
        last_cursor_x_ = shoot_x_;
        last_cursor_y_ = shoot_y_;
        data_[9]  = yaw_deg_;
        data_[10] = pitch_deg_;
        RCLCPP_INFO(this->get_logger(),
            "/manual/basket: cell=%d -> [%d][%d](x,y) yaw=%d pitch=%d injection_speed=%d",
            idx, shoot_x_, shoot_y_, yaw_deg_, pitch_deg_, current_injection_speed());
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

        // 現在モードをGUIへ publish (0=Drive, 1=Get_eel, 2=shoot_mode)
        std_msgs::msg::String mode_msg;
        switch (g_mode_count.load() % 3) {
            case 0:  mode_msg.data = "DRIVE";   break;
            case 1:  mode_msg.data = "GET_EEL"; break;
            default: mode_msg.data = "SHOOT";  break;
        }
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
    //             "[マイクロSW] 上(SHARE禁止用)=%s  下(R1禁止用)=%s",
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
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr basket_sub_; // /manual/basket (GUIカゴ指定)

    // SHOOTモードのカーソル位置。プリセット参照は kYawPreset[shoot_x_][shoot_y_] で、
    // 起動時は [0][2] から開始。dpadとGUIカゴ指定で共有。
    int shoot_x_ = 0;
    int shoot_y_ = 2;

    // 照準サーボの現在角。yaw=data_[9], pitch=data_[10] に毎周期そのまま入る。
    // 「今どこを向いているか」を持つのはこの2つだけ(射出モードや走行モードは持たない)。
    //   射出モード : カゴを選んだ瞬間だけプリセット角を代入する
    //   走行モード : 十字キーで ±3度ずつ微調整する
    //   モード切替 : 誰も書かないので角度はそのまま残る
    int yaw_deg_ = 0;     // 0〜270度
    int pitch_deg_ = 270; // 90〜270度

    // プリセット代入を「カゴを選び直した瞬間」だけにするための前回カーソル位置。
    // -1 始まりなので、射出モードに最初に入った時は必ず一度プリセットが入る。
    int last_cursor_x_ = -1;
    int last_cursor_y_ = -1;

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