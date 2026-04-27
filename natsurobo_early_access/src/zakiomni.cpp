#include "zakiomni.hpp"

    Zakicar::Zakicar() : Node("omni_drive") {

        std::cout << "Waiting to receive topics." << std::endl;

        //マイコンにトピック（モーター）を送信
        motor_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_1", 10);
        //マイコンからトピック（エンコーダの値）を受信
        enc_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_1", 10, std::bind(&Zakicar::encoderCallback, this, std::placeholders::_1));
        
        //joyスティックからトピックを受信
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy",
             10,
         std::bind(&Zakicar::ps4callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&Zakicar::About_PID,this));//20msごとにPID制御の関数を呼び出す
    }

    void Zakicar::encoderCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        rclcpp::Time current = this->get_clock()->now();//何故か知らないけど間にget_clock()挟まないとコンパイルエラーと化した
        dt = (current - last).seconds();

        //セーフティチェック(通信)
            if(!enc_received) {
                for(int i = 0; i < 4; i++){
                    pre_enc[i] = static_cast<uint16_t>(msg->data[i+1]);
                }
                enc_received = true;
                last = current;
                dt = 0.0;
                return;
            } // last,pre_encの初期化
            if(dt <= 0.0) {//dtが0かあまりに小さいと計算に使えるか怪しいのでなかったコトにしてreturn
                last = current;
                return;
            } else if(dt <= 0.005){
                return;
            }// 申し訳ないが初期のdt（=0)はNG
            if(!shivangelion_activated){
                shivangelion();
            }

            //エンコーダのオーバーフローを防止と回転数の計算
            for(int i = 0; i < 4; i++){
                enc_data_[i]= msg->data[i+1];
                now_enc[i] = enc_data_[i];
                pre_enc32[i] = static_cast<int16_t>(pre_enc[i]);//計算の都合上、型を変える
                diff32[i] = now_enc[i] - pre_enc32[i]; 
                if(diff32[i] > enc_max/2) {
                    diff32[i] -= enc_max; 
                } else if (diff32[i] < -enc_max/2) {
                    diff32[i] += enc_max; 
                }
                diff[i] = static_cast<int16_t>(diff32[i]);
                zakirps[i] = -diff[i]/(dt * cpr); // 回転数を計算(-は回転方向の調整)
                last = current;
                pre_enc[i] = now_enc[i];
            }

            enc_data_[3] = zaki * enc_data_[3];
    }
    
    void Zakicar::ps4callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        LS_Y = msg->axes[1];
        LS_X = -1*msg->axes[0];
        //RS_X = -1 * msg->axes[3];
        //RS_Y = msg->axes[4];
        //CROSS = msg->buttons[0];
        //CIRCLE = msg->buttons[1];
        //TRIANGLE = msg->buttons[2];
        //SQUARE = msg->buttons[3];
        //LEFT = msg->axes[6] == 1.0;
        //RIGHT = msg->axes[6] == -1.0;
        //UP = msg->axes[7] == 1.0;
        //DOWN = msg->axes[7] == -1.0;
        //L1 = msg->buttons[4];    
        //R1 = msg->buttons[5];
        //L2 = (-1 * msg->axes[2] + 1) / 2;
        //R2 = (-1 * msg->axes[5] + 1) / 2;

        //bool SHARE = msg->buttons[8];
        //bool OPTION = msg->buttons[9];
        //bool PS = msg->buttons[10];

        //bool L3 = msg->buttons[11];
        //bool R3 = msg->buttons[12];
        
        //セーフティチェック(スティック)
        if(fabs(LS_Y) < DEADZONE_L) {
            LS_Y = 0.0f; //十分小さいのでゼロとみなす
        }else if(fabs(LS_X) < DEADZONE_L){
            LS_X = 0.0f;
        }

        radian = atan2(LS_Y, LS_X); //スティックの角度を算出

        target_v[0] = max_target_cps * std::cos(radian + (opPI / 4) ); // スティックの入力に基づいて目標速度を計算 <-しかし一輪しかない
        target_v[1] = max_target_cps * std::cos(radian - (opPI / 4) );
        target_v[2] = max_target_cps * -std::cos(radian + (opPI / 4) );
        target_v[3] = max_target_cps * -std::cos((opPI /4) - radian );

        joy_received = true;//joystick受信フラグ
        last_joy_time = this->get_clock()->now();
        }
    void Zakicar::About_PID(){

        // セーフティチェック(joy)
        if(!joy_received){
            for(int i = 0; i < 4; i++){
               target_v[i] = 0.0;
            }
            return;
        }
        double blank_time = (this->get_clock()->now() - last_joy_time).seconds();//joyとの通信間隔
        if(blank_time > 1.0) {//申し訳ないが1秒以上入力しないとタイムアウトして速度をゼロにする
            for(int j = 0; j < 4; j++){
                target_v[j] = 0.0;
            }
           joy_received = false; //ジョイスティックの入力がない状態に戻す
           }

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

        auto feedback = std_msgs::msg::Int16MultiArray();
        feedback.data.assign(25, 0);//受信側のサイズが固定されてるので;
        for(int o = 0; o < 4; o++){
            feedback.data[o+1] = zakipow[o];
            last_zakipow[o] = zakipow[o];
        }
        motor_pub_->publish(feedback);

        // デバッグ用のログ出力
        RCLCPP_INFO(this->get_logger(),
                "dt: %f, Enc : %d,LS_Y: %f,θ:%f, %frps,power: %d,T_v: %f,P: %f,I: %f",
                dt,enc_data_[0], LS_Y, radian, zakirps[0], zakipow[0], target_v[0], P[0], I[0]);//現状一輪しかないので

    };
    void Zakicar::shivangelion(){

        /////////////おふざけ//////////////
        if(!shivangelion_activated){
            const char* msg = " Shivangelion!!! Activatation!!!";
            std::string fig_msg = "figlet " + std::string(msg);
            std::system(fig_msg.c_str());
            shivangelion_activated = true;
        }
        /////////////////////////////////
    }

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zakicar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}