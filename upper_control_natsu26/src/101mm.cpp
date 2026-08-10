/*
Serial_Bridgeノードのホスト側プログラム
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/

#include "101mm.hpp"


    unaginobori2026::unaginobori2026(uint8_t tx_device_id)
        : Node("unaginobori2026"),
          tx_device_id_(tx_device_id)
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
            std::bind(&unaginobori2026::ps4_listener_callback, this, std::placeholders::_1));

        // seial_bridgeへpublish
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        // 自動昇降: シリンダ上げ保持時間[s]
        raise_sec_ = this->declare_parameter<double>("raise_sec", 2.0);

        // natsu_auto の CLIMB から /climb/start を受けて昇降、完了で /climb/done を返す
        climb_start_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/climb/start", 10,
            std::bind(&unaginobori2026::climb_start_callback, this, std::placeholders::_1));
        climb_done_pub_ = this->create_publisher<std_msgs::msg::Bool>("/climb/done", 10);

        // timer_callbackを呼び出すタイマーを作成
        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&unaginobori2026::publisher_timer_callback, this));

        // sensor_callbackは廃止しました。

        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
    }


    void unaginobori2026::ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {

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
        // bool UP = msg->axes[7] == 1.0;
        // bool DOWN = msg->axes[7] == -1.0;

        bool L1 = msg->buttons[4];
        // bool R1 = msg->buttons[5];

        float L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        // float R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // bool L2 = msg->buttons[6];
        // bool R2 = msg->buttons[7];

        // bool SHARE = msg->buttons[8];
        // bool OPTION = msg->buttons[9];
        // bool PS = msg->buttons[10];

        // bool L3 = msg->buttons[11];
        // bool R3 = msg->buttons[12];

        #if defined(CRANEGAME)
            bool TRIANGLE = msg->buttons[2];
            bool CROSS = msg->buttons[0];
            bool SQUARE = msg->buttons[3];
        #endif

        // 以降、配列data_を操作する

        if (L1 && !last_L1)
        {
            L1_count++;
        }

        if (drive_mode)
        {
            // RCLCPP_INFO(this->get_logger(),
            //                     "Mode:Drive.");
            // ドライブモードの処理

            if (CIRCLE && !last_CIRCLE)
            { // CIRCLEが押されたときに一度だけ実行される処理（CIRCLEを押すたびに段差超え処理を進める）
                // 自動化出来るか分からんから一応完全マニュアル操作を想定

                if (CIRCLE_count % 3 == 0)
                {
                    // 4輪をエアシリンダで上げる処理（段差超え）
                    data_[17] = 1; // 前輪
                    data_[18] = 1; // 後輪
                    CIRCLE_count++;
                }
                else if (CIRCLE_count % 3 == 1)
                {
                    data_[17] = 0;
                    data_[18] = 1; // 前輪格納（手動で前進してね^^）
                    CIRCLE_count++;
                }
                else if (CIRCLE_count % 3 == 2)
                {
                    data_[17] = 0;
                    data_[18] = 0; // 後輪格納
                    CIRCLE_count++;
                }
            }

            if(L2_DIGITAL > 0.1){ 
                // 前進用ホイールのモータの番号data_[1~2];
                data_[1] = -95 * L2_DIGITAL; 
                data_[2] = 95 * L2_DIGITAL;
            
            }else {
                data_[1] = 0;
                data_[2] = 0;

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
            // last_R1 = R1;

            /*if(DOWN) {
                //前進用ホイールのモータの番号data_[1~4] = -15;
                //前進用ホイールのモータの番号data_[1~4] = -15;
            }*/

            // デバッグ用
            // RCLCPP_INFO(
            //     get_logger(),
            //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]"
            //     data_[1], data_[2], data_[3], data_[4],
            //     data_[9], data_[10], data_[11], data_[12]);
            last_CIRCLE = CIRCLE;
            last_L1 = L1;

        }
        else if (get_eel_mode)
        {
            #if defined(CRANEGAME)
                if(SQUARE && last_SQUARE){
                    if(SQUARE_count % 4 == 0){
                        //data_[9から16のどっか] = cranegame_servo();
                    }
                    else if(SQUARE_count % 4 == 1){
                        //data_[9から16のどっか] = cranegame_tr();
                    }else if(SQUARE_count % 4 == 2){
                        //data_[9から16のどっか] = cranegame_tr();
                    }
                    else if(SQUARE_count % 4 == 3){
                        //data_[9から16のどっか] = cranegame_servo();
                    }
                    SQUARE_count++;
                }
                // data_[1から8のどっか] = cranegame_motor();
                
            #endif
        }

        

        // 配列操作ここまで
    }

    // 自動昇降開始: /climb/start(data=true) を受けたらエアシリンダを上げてシーケンス開始。
    // 実行中の再受信は無視(多重起動防止)。
    void unaginobori2026::climb_start_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data || climbing_)
            return;

        data_[17] = 1; // 前輪シリンダ上げ
        data_[18] = 1; // 後輪シリンダ上げ
        climbing_ = true;
        climb_start_time_ = this->now();
        RCLCPP_INFO(get_logger(), "自動昇降を開始します: シリンダ上げ (raise_sec=%.1fs)", raise_sec_);
    }

    // publish
    void unaginobori2026::publisher_timer_callback()
    {
        // 自動昇降シーケンスの完了判定: raise_sec_ 経過で /climb/done を1回発行
        if (climbing_ &&
            (this->now() - climb_start_time_).seconds() >= raise_sec_)
        {
            climbing_ = false;
            std_msgs::msg::Bool done;
            done.data = true;
            climb_done_pub_->publish(done);
            RCLCPP_INFO(get_logger(), "自動昇降が完了しました → /climb/done を発行");
        }

        std_msgs::msg::Int16MultiArray msg;

        // 一応...
        RCLCPP_INFO(
            this->get_logger(),
            "Mode:%s, Phase:%d data_[17]=%d, data_[18]=%d, motor=[%d,%d]",
            L1_count % 2 == 0 ? "Drive" : "Get_Eel", (CIRCLE_count % 3) + 1, data_[17], data_[18], data_[1], data_[2]); // data_[1~4], data_[1~4]);

        msg.data = data_;

        publisher_->publish(msg);
    }

int main(int argc, char *argv[])
{

    rclcpp::init(argc, argv);

    // figletでノード名を表示
    std::string figletout = "figlet Air 2026";
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

    auto upper_control = std::make_shared<unaginobori2026>(TX_DEVICE_ID);
    exec.add_node(upper_control);
    exec.spin();

    rclcpp::shutdown();
    return 0;
};