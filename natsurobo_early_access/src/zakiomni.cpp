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
        | data[18] | TR2 | 0 or 1|        
        | data[19] | TR3 | 0 or 1|
        | data[20] | TR4 | 0 or 1|
        | data[21] | TR5 | 0 or 1|
        | data[22] | TR6 | 0 or 1|
        | data[23] | TR7 | 0 or 1|
        | data[24] | TR8 | 0 or 1|
        */
       //rclcpp::Time current = this->get_clock()->now();//何故か知らないけど間にget_clock()挟まないとコンパイルエラーと化した<-なんか消してもビルド通った

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
            std::bind(&Zakicar::publisher_timer_callback, this));//マイコンと基板の関係でpublisher_とsensor_sub_のトピックは別です(pub:serial_tx_2 , sub:serial_rx_1にする予定)

        //マイコンからトピック（エンコーダの値）を受信
        sensor_sub_ = this->create_subscription<std_msgs::msg::Int16MultiArray>(
            "serial_rx_" + std::to_string(rx_device_id_), 
            10, 
            std::bind(&Zakicar::sensor_callback, 
                  this, 
                  std::placeholders::_1));
        
        RCLCPP_INFO(get_logger(),
                    "serial_tx_%d started.", tx_device_id_);
        RCLCPP_INFO(get_logger(),
                    "serial_rx_%d started.", rx_device_id_);//tx =rxではないので
        
    }

    void Zakicar::ps4_listener_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        
        
        // コントローラーの入力を取得、使わない入力はコメントアウト推奨
        LS_X = -1 * msg->axes[0];
        LS_Y = msg->axes[1];
        RS_X = -1 * msg->axes[3];
        // RS_Y = msg->axes[4];

        // CROSS = msg->buttons[0];
        // CIRCLE = msg->buttons[1];
        // TRIANGLE = msg->buttons[2];
        // SQUARE = msg->buttons[3];

        // LEFT = msg->axes[6] == 1.0;
        // RIGHT = msg->axes[6] == -1.0;
        // UP = msg->axes[7] == 1.0;
        // DOWN = msg->axes[7] == -1.0;

        // L1 = msg->buttons[4];
        // R1 = msg->buttons[5];

        // L2_DIGITAL = (-1 * msg->axes[2] + 1) / 2;
        R2_DIGITAL = (-1 * msg->axes[5] + 1) / 2;

        // L2 = msg->buttons[6];
        // R2 = msg->buttons[7];

        // SHARE = msg->buttons[8];
        // OPTION = msg->buttons[9];
        // PS = msg->buttons[10];

        // L3 = msg->buttons[11];
        // R3 = msg->buttons[12];

        // last_option = false;
        // option_latch = false;

        // last_share = false;
        // share_latch = false;

        // 以降、配列data_を操作する

        // デバッグ用
        // RCLCPP_INFO(
        //     get_logger(),
        //     "data_[1-4]=[%d,%d,%d,%d], data_[9-12]=[%d,%d,%d,%d]",
        //     data_[1], data_[2], data_[3], data_[4],
        //     data_[9], data_[10], data_[11], data_[12]);

        if(fabs(LS_X) <= DEADZONE_L)
            LS_X = 0.0;
        if(fabs(LS_Y) <= DEADZONE_L)
            LS_Y = 0.0;
        if(fabs(RS_X) <= DEADZONE_R)
            RS_X = 0.0;
        radian = atan2(LS_Y, LS_X); //スティックの角度を算出
        for(int i = 0; i < 4; i++){
            target_v[i] = 0.0;//初期化（これがないとスティックを元に戻しても0にならない）
        }

        //直進モード(R2のみを押したとき)
        if(R2_DIGITAL && (LS_X == 0.0 && LS_Y == 0.0) ) {
            radian = opPI / 2.0;
            target_v[0] = max_target_move_cps * R2_DIGITAL * std::cos((3.0/4.0 * opPI) - radian); 
            target_v[1] = max_target_move_cps * R2_DIGITAL * std::cos((opPI /4.0) - radian);
            target_v[2] = max_target_move_cps * R2_DIGITAL * std::cos(radian + (opPI /4.0));
            target_v[3] = max_target_move_cps * R2_DIGITAL * -std::cos((opPI /4.0) - radian);
        }

        //移動モード(R2を押し込みながら)
        if(LS_X || LS_Y){
            target_v[0] += max_target_move_cps * R2_DIGITAL * std::cos((3.0/4.0 * opPI) - radian); // スティックの入力に基づいて正射影を求め、モーターの出力方向に変換
            target_v[1] += max_target_move_cps * R2_DIGITAL * std::cos((opPI /4.0) - radian);
            target_v[2] += max_target_move_cps * R2_DIGITAL * std::cos(radian + (opPI /4.0));
            target_v[3] += max_target_move_cps * R2_DIGITAL * -std::cos((opPI /4.0) - radian);
        }
        
        //旋回モード
        if(RS_X){
            for(int i = 0; i < 4; i++){
                target_v[i] = -max_target_yaw_cps * RS_X;
            }
           
        }
        for (int l = 0; l < 4; l++) {
            target_v[l] = filter * target_v[l] + (1.0 - filter) * last_target_v[l];//低速帯の振動が激しいため、AIに書かせたけど割と優秀
            last_target_v[l] = target_v[l];  
        }

            // 配列操作ここまで

            joy_received = true;//joystick受信フラグ
            last_joy_time = this->now();
            
        }

    void Zakicar::publisher_timer_callback() {

        about_PID();//一定周期でtimerが呼び出されるときに連動してActivate!

        if(!shivangelion_activated && joy_received){//デバック用
            Shivangelion();
        }

        std_msgs::msg::Int16MultiArray msg;
        msg.data = data_; 
        publisher_->publish(msg); //送信
    }

    void Zakicar::sensor_callback(const std_msgs::msg::Int16MultiArray::SharedPtr msg) {
        
        current = this->now();
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

        if(dt <= 0.0) {// 申し訳ないが初期のdt（=0)はNG          
            last = current;
            return;
        }
        if(dt <= 0.005){//dtが0かあまりに小さいと計算に使えるか怪しいのでなかったコトにしてreturn
            return;
        }      
            
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

        // 受信データ処理ここまで
    }
    
    
    void Zakicar::about_PID(){

        
        //セーフティチェック（joy）
        if(!joy_received){
            for(int i = 0; i < 4; i++){
               target_v[i] = 0.0;
            }
            return;
        }

        //セーフティチェック（joy）その2
        double blank_time = (this->now() - last_joy_time).seconds();//joyとの通信間隔        

        if(blank_time > 1.0) {//申し訳ないが1秒以上入力しないとタイムアウトして速度をゼロにする
            for(int j = 0; j < 4; j++){
                target_v[j] = 0.0;
            }
           joy_received = false; //ジョイスティックの入力がない状態に戻す
        }
        
        for(int k = 0; k < 4; k++){
            err[k] = target_v[k] - zakirps[k];//P制御
            err_sum[k]  += err[k] * dt; //I制御
            err_diff[k] = (err[k] - last_err[k]) / dt; //D制御
        }

        // PI制御の出力を計算
        for(int l = 0; l < 4; l++){
            FF[l] = Kff * target_v[l]; // フィードフォワード(PIだけじゃ出力がしょぼすぎたから書いたけど結局いらなかったかも)
            P[l] = Kp * err[l];
            I[l] = std::clamp(Ki * err_sum[l], -Imax, Imax); // -Imax <= err_sum <= Imaxに制限
            D[l] =  Kd * err_diff[l];

            if(fabs(target_v[l]) <= 5.0 ) {//低速ではPのみで十分かなって
                I[l] = 0.0;
                D[l] = 0.0;
            }
            motor_power[l] = FF[l] + P[l] + I[l] + D[l];
            last_err[l] = err[l];
        }

        // 目標速度がほぼ0の時は停止したいから蓄積をリセット
        for(int m = 0; m < 4; m++){
            if (fabs(target_v[m]) <= 0.3) {
                err_sum[m] = 0.0;
                motor_power[m] = 0.0;
            }
        }

        std::swap(motor_power[0], motor_power[2]);//モーターの配置に合わせて入れ替え
        std::swap(motor_power[1], motor_power[3]);

        for(int n = 0; n < 4; n++){
            data_[n+1] = std::clamp(motor_power[n],-motor_limit,motor_limit); // モーターの出力を制限        
            data_[n+1] = std::clamp<int16_t>(data_[n+1], last_data_[n] - delta_power_limit, last_data_[n] + delta_power_limit); // 出力の変化を制限)
            
            last_data_[n] = data_[n+1];
        }

        data_[2] = 0;//data_[2](4)番が物理的に故障しました

        // デバッグ用のログ出力
        RCLCPP_INFO(this->get_logger(),
                "dt: %f,T_v[1-4]: %f,%f,%f,%f,rps[1-4]: %f,%f,%f,%f,"
                "power[1-4]: %d,%d,%d,%d,P[1-4]: %f,%f,%f,%f,I[1-4]: %f,%f,%f,%f,"/*D[1-4]: %f,%f,%f,%f,KFF: %f*///使ってないからコメントだけ
            
                ,dt, target_v[0],target_v[1],target_v[2],target_v[3], zakirps[0],zakirps[1],zakirps[2],zakirps[3],
                data_[3],data_[4],data_[1],data_[2], P[0],P[1],P[2],P[3], I[0],I[1],I[2],I[3]/*,D[0],D[1],D[2],D[3],Kff*/);

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