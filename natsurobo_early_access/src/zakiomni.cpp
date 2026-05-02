#include "zakiomni.hpp"

    Zakicar::Zakicar(uint8_t tx_device_id, uint8_t rx_device_id) 
    : Node("omni_drive"), tx_device_id_(tx_device_id), rx_device_id_(rx_device_id) {
 //: Node("hardware_control_" + std::to_string(tx_device_id)),

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
        | data[18] | TR2 | 0 or 1|        rclcpp::Time current = this->get_clock()->now();//何故か知らないけど間にget_clock()挟まないとコンパイルエラーと化した
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */

        std::cout << "Waiting to receive topics." << std::endl;//なにもトピックを受け取ってないときの状態が欲しかった

        //joyスティックからトピックを受信
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
             10,
         std::bind(&Zakicar::ps4_listener_callback, this, std::placeholders::_1));

        //マイコンにトピック（モーター）を送信
        publisher_ = this->create_publisher<std_msgs::msg::Int16MultiArray>(
            "serial_tx_" + std::to_string(tx_device_id_), 10);

        timer_ = create_wall_timer(
            std::chrono::milliseconds(PUBLISH_RATE_MS),
            std::bind(&Zakicar::publisher_timer_callback, this));

        //マイコンからトピック（エンコーダの値）を受信
        snsor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_" + std::to_string(rx_device_id_), 
        10, 
        std::bind(&Zakicar::sensor_callback, 
                  this, 
                  std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
        
    }

    void Zakicar::ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        
        
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        LS_X = -1 * msg->axes[0];
        LS_Y = msg->axes[1];
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

        // 以降、配列data_を操作する

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);

        //セーフティチェック（joy）
        if(!joy_received)
            for(int i = 0; i < 4; i++){
               target_v[i] = 0.0;
            return;
        }

        double blank_time = (this->get_clock()->now() - last_joy_time).seconds();//joyとの通信間隔
        if(blank_time > 1.0) {//申し訳ないが1秒以上入力しないとタイムアウトして速度をゼロにする
            for(int j = 0; j < 4; j++){
                target_v[j] = 0.0;
            }
           joy_received = false; //ジョイスティックの入力がない状態に戻す
           }

        radian = atan2(LS_Y, LS_X); //スティックの角度を算出
        angle = radian * 180 / opPI; //角度を度数法に変換（デバッグ用） 

        target_v[0] = max_target_cps * sqrt((pow(LS_X ,2) + pow(LS_Y ,2)) / 2) * std::cos( (3 * opPI / 4) - radian); // スティックの入力に基づいて目標速度を計算 <-しかし一輪しかない
        target_v[1] = max_target_cps * sqrt((pow(LS_X ,2) + pow(LS_Y ,2)) / 2) * -std::cos( radian + (3 * opPI / 4) );
        target_v[2] = max_target_cps * sqrt((pow(LS_X ,2) + pow(LS_Y ,2)) / 2) * -std::cos(radian + (opPI / 4) );
        target_v[3] = max_target_cps * sqrt((pow(LS_X ,2) + pow(LS_Y ,2)) / 2) * std::cos( (opPI / 4) - radian );

        // 配列操作ここまで

        joy_received = true;//joystick受信フラグ
        last_joy_time = this->get_clock()->now();
        }

    void Zakicar::publisher_timer_callback() {

        about_PID();

        if(!shivangelion_activated && joy_received){
            Shivangelion();
        }

        std_msgs::msg::Int16MultiArray msg;
        msg.data = data_; // 送信するデータをセット
        publisher_->publish(msg); // トピックを発行
    }

    void Zakicar::sensor_callback(
        const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        current = this->get_clock()->now();
        dt = (current - last).seconds();

        ENC1 = msg->data[1];
        ENC2 = msg->data[2];
        ENC3 = msg->data[3];
        ENC4 = msg->data[4];
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

        if(!enc_received) {//初回限定初期化

            for(int i = 0; i < 4; i++){
                pre_enc[i] = static_cast<uint16_t>(msg->data[i+1]);//こいつだけここに配置するしかなかった
            }
            enc_received = true;
            last = current;
            dt = 0.0;
            return;
        } 

        //エンコーダのオーバーフローを防止と回転数の計算

        if(dt <= 0.0) {//dtが0かあまりに小さいと計算に使えるか怪しいのでなかったコトにしてreturn
                last = current;
                return;
            } else if(dt <= 0.005){
                return;
            }// 申し訳ないが初期のdt（=0)はNG                
            
            diff32[0] =  ENC1- pre_enc32[0]; 
            diff32[1] =  ENC2- pre_enc32[1];
            diff32[2] =  ENC3- pre_enc32[2];
            diff32[3] =  ENC4- pre_enc32[3];

            for(int i = 0; i < 4; i++){
                pre_enc32[i] = static_cast<int16_t>(pre_enc[i]);//計算の都合上、型を変える
                if(diff32[i] > enc_max/2) {
                    diff32[i] -= enc_max; 
                } else if (diff32[i] < -enc_max/2) {
                    diff32[i] += enc_max; 
                }
                diff[i] = static_cast<int16_t>(diff32[i]);
                zakirps[i] = -diff[i]/(dt * cpr); // 回転数を計算(-は回転方向の調整)
                }
            last = current;
            pre_enc[0] = ENC1;
            pre_enc[1] = ENC2;
            pre_enc[2] = ENC3;
            pre_enc[3] = ENC4;
            
            ENC1 = zaki * ENC1;
            ENC2 = zaki * ENC2;//実機とのギャップを解消
            ENC4 = zaki * ENC4;

        // 受信データ処理ここまで
    }
    
    
    void Zakicar::about_PID(){

        auto msg = std_msgs::msg::Int16MultiArray();
        

        for(int k = 0; k < 4; k++){
            err[k] = target_v[k] - zakirps[k];//P制御
            err_sum[k]  += err[k] * dt; //I制御
        }

        // PI制御の出力を計算
        for(int l = 0; l < 4; l++){
            P[l] = Kp * err[l];
            I[l] = std::clamp(Ki * err_sum[l], -Imax, Imax); // -Imax <= err_sum <= Imaxに制限
            motor_power[l] = P[l] + I[l];
        }

        // 目標速度が0の時は停止したいから蓄積をリセット
        for(int m = 0; m < 4; m++){
            if (fabs(target_v[m]) <= 0.1) {
                err_sum[m] = 0.0;
                motor_power[m] = 0.0;
            }
        }

        for(int n = 0; n < 4; n++){
            zakipow[n] = static_cast<int16_t>(std::clamp(motor_power[n],-motor_limit,motor_limit)); // モーターの出力を制限        
            zakipow[n] = std::clamp(zakipow[n], last_zakipow[n] - delta_power_limit, last_zakipow[n] + delta_power_limit); // 出力の変化を制限
        }

        for(int o = 0; o < 4; o++){
            data_[o+1] = zakipow[o];
            last_zakipow[o] = zakipow[o];
        }
        msg.data = this->data_;
        publisher_->publish(msg);//一旦送っちゃおう

        // デバッグ用のログ出力
        RCLCPP_INFO(this->get_logger(),
                "dt: %f,Enc[1-4] : %d,%d,%d,%d,LS_X: %f,LS_Y: %f,θ: %f,rps[1-4]: %f,%f,%f,%f,power[1-4]: %d,%d,%d,%d,"
                "T_v[1-4]: %f,%f,%f,%f,P[1-4]: %f,%f,%f,%f,I[1-4]: %f,%f,%f,%f",
                dt,ENC1,ENC2,ENC3,ENC4,LS_X, LS_Y, angle, zakirps[0],zakirps[1],zakirps[2],zakirps[3],
                zakipow[0],zakipow[1],zakipow[2],zakipow[3], target_v[0],target_v[1],target_v[2],target_v[3], P[0],P[1],P[2],P[3], I[0],I[1],I[2],I[3]);//現状一輪しかないので

    };
    void Zakicar::Shivangelion(){

        if(!shivangelion_activated){
            const char* msg = " Shivangelion!!! Activatation!!!";
            std::string fig_msg = "figlet " + std::string(msg);
        int result = std::system(fig_msg.c_str());

        if (result != 0) {
            std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                      << std::endl;
            std::cerr << "Please install 'figlet' with the following command:"
                      << std::endl;
            std::cerr << "sudo apt install figlet" << std::endl;
            std::cerr << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
                      << std::endl;
        }

            shivangelion_activated = true;
        }

    }

int main(int argc, char *argv[]) {

    rclcpp::init(argc, argv);

    rclcpp::executors::MultiThreadedExecutor exec;

    auto zakicar = std::make_shared<Zakicar>(TX_DEVICE_ID, RX_DEVICE_ID);
    exec.add_node(zakicar);
    exec.spin();

    rclcpp::shutdown();
    return 0;
}