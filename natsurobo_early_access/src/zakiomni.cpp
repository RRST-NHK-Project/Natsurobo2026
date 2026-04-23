#include "zakiomni.hpp"

    Zakicar::Zakicar() : Node("OmniDrive") {

        ////////////おふざけ//////////////
        const char* msg = " Shivangelion!!! Activatation!!!";
        std::string fig_msg = "figlet " + std::string(msg);
        std::system(fig_msg.c_str());
        /////////////////////////////////

        //マイコンにトピック（モーター）を送信
        motor_pub_ = this->create_publisher<std_msgs::msg::Int16MultiArray>("serial_tx_1", 10);
        //マイコンからトピック（エンコーダの値）を受信
        enc_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_1", 10, std::bind(&Zakicar::encoderCallback, this, std::placeholders::_1));
        
        //joyスティックからトピックを受信
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 10, std::bind(&Zakicar::PS4Callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(
            20ms,
            std::bind(&Zakicar::About_PID,this));//20msごとにPID制御の関数を呼び出す
    }

    void Zakicar::encoderCallback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        rclcpp::Time current = this->now();

        //セーフティチェック(通信)
        if(!enc_received) {
            pre_enc = static_cast<uint16_t>(msg->data[1]);
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

        //エンコーダのオーバーフローを防止と回転数の計算
        enc_data_ = msg->data[1];
        int32_t now_enc = enc_data_,pre_enc32 = static_cast<int16_t>(pre_enc);//計算の都合上、型を変える
        int32_t diff32 = now_enc - pre_enc32; 
        if(diff32 > enc_max/2) {
            diff32 -= enc_max; 
        } else if (diff32 < -enc_max/2) {
            diff32 += enc_max; 
        }
        int16_t diff = static_cast<int16_t>(diff32);
        rps = diff/(dt * cpr); // 回転数を計算
        last = current;
        pre_enc = now_enc;
    }
    
    void Zakicar::PS4Callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        LS_Y = msg->axes[1];
        //LS_X = -1*msg->axes[0];
        //LS_Y = msg->axes[1];
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
        }

        target_v = LS_Y * max_target_cps; // スティックの入力に基づいて目標速度を計算
        joy_received = true;//joystick受信フラグ
        last_joy_time = this->now();
        }
    void Zakicar::About_PID(){

        // セーフティチェック(joy)
        if(!joy_received){
            target_v = 0.0; 
            return;
        }
        double blank_time = (this->now() - last_joy_time).seconds();//joyとの通信間隔
        if(blank_time > 1.0) {//申し訳ないが1秒以上入力しないとタイムアウトして速度をゼロにする
            target_v = 0.0; 
            joy_received = false; //ジョイスティックの入力がない状態に戻す
        }


        err = target_v - rps;//P制御
        err_sum += err * dt; //I制御

        // PI制御の出力を計算
        double P = Kp * err;
        double I = Ki * err_sum;

        I = std::clamp(I, -Imax, Imax);// -Imax <= err_sum <= Imaxに制限

        double motor_power = P + I;

        // 目標速度が0の時は停止したいから蓄積をリセット
        if (target_v == 0.0) {
            err_sum = 0.0;
            motor_power = 0.0;
        }

        zakistep = static_cast<int16_t>(std::clamp(motor_power,-motor_limit,motor_limit)); // モーターの出力を制限        
        zakistep = std::clamp(zakistep, last_zakistep - delta_power_limit, last_zakistep + delta_power_limit); // 出力の変化を制限
            
        auto feedback = std_msgs::msg::Int16MultiArray();
        feedback.data.assign(25, 0);//受信側のサイズが固定されてるので;
        feedback.data[1] = zakistep; 
        motor_pub_->publish(feedback);//serial_tx_1(モーター)に値を送る
        last_zakistep = zakistep;

        // デバッグ用のログ出力
        RCLCPP_INFO(this->get_logger(),
                "dt: %f, Enc : %d,LS_Y: %f, %frps,power: %d,T_v: %f,P: %f,I: %f",
                dt,enc_data_, LS_Y, rps, zakistep, target_v, P, I);

    };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zakicar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}