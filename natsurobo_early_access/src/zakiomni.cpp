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
        if(!enc_received) {
            pre_enc = static_cast<uint16_t>(msg->data[1]);
            enc_received = true;
            last = current;
            return;
        } // last,pre_encの初期化
       
        double dt = (current - last).seconds();//rosの時間を.seconds()で秒に変換
        if(dt <= 0.0) {
            last = current;
            return;
        } // 申し訳ないが初期のdt（=0)はNG
        enc_data_ = msg->data[1];
        uint16_t now_enc = enc_data_;//計算の都合上、型を変える
        uint16_t u_diff = now_enc - pre_enc;
        int16_t diff = static_cast<int16_t>(u_diff); //計算が終われば元に戻す
        current = this->now();
        pre_enc = now_enc;
        zakiomni_v = diff / dt; // 速度を計算
        last = current;
    }
    
    void Zakicar::PS4Callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
    // コントローラーの入力を取得、使わない入力はコメントアウト推奨
    LS_Y = msg->axes[1];
    //LS_X = -1*msg->axes[0];
    //LS_Y = msg->axes[1];
    //RS_X = -1 * msg->axes[3];
    //  RS_Y = msg->axes[4];
    // CROSS = msg->buttons[0];
    // CIRCLE = msg->buttons[1];
    // TRIANGLE = msg->buttons[2];
    // SQUARE = msg->buttons[3];
    // LEFT = msg->axes[6] == 1.0;
    //RIGHT = msg->axes[6] == -1.0;
    //UP = msg->axes[7] == 1.0;
    //DOWN = msg->axes[7] == -1.0;
    // L1 = msg->buttons[4];    
    // R1 = msg->buttons[5];
    //  L2 = (-1 * msg->axes[2] + 1) / 2;
    //R2 = (-1 * msg->axes[5] + 1) / 2;

    //bool SHARE = msg->buttons[8];
    //bool OPTION = msg->buttons[9];
    // bool PS = msg->buttons[10];

    //bool L3 = msg->buttons[11];
    //bool R3 = msg->buttons[12];
        joy_received = true;//joystick受信フラグ
        last_joy_time = this->now();
        
        if(fabs(LS_Y) < DEADZONE_L) {
            LS_Y = 0.0f; //十分小さいのでゼロとみなす
        }
        target_v =static_cast<double>(LS_Y) * max_target_cps; // スティックの入力に基づいて目標速度を計算
        }
    void Zakicar::About_PID(){
        if(!joy_received){
            target_v = 0.0; 
            return;
        }
        double blank_time = (this->now() - last_joy_time).seconds();//現在時刻と最後にジョイスティックを受け取った時刻の差
        if(blank_time > 1.0) {//申し訳ないが1秒以上入力しないとタイムアウトして速度をゼロにする
            target_v = 0.0; 
            joy_received = false; //ジョイスティックの入力がない状態に戻す
        }
        rpm = (zakiomni_v / cpr) * 60.0; // 回転数を計算
        err = target_v - zakiomni_v;
        auto feedback = std_msgs::msg::Int16MultiArray();
        feedback.data.assign(25, 0);//受信側のサイズが固定されてるので;
        zakistep = std::clamp(Kp*err, -max_target_cps, max_target_cps);
        feedback.data[1] = zakistep; 
        motor_pub_->publish(feedback);//serial_tx_1(モーター)に値を送る
          RCLCPP_INFO(this->get_logger(), "Enc : %d,LS_Y: %f,Probably, %frpm,PID: %d", enc_data_, LS_Y, rpm, zakistep);

    };

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Zakicar>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}