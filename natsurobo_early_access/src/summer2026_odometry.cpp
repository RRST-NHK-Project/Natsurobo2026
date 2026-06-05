#include "summer2026_odometry.hpp"



/*
hardware_control_2を起動するとオドメトリの自己位置を計算し、tfも送信する（但し現状では無意味）。
mc_2026.cppが死ぬとこいつも共倒れする
3輪オドメトリの回転をエンコーダから受け取って位置を計算する
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/
Shivalian_control::Shivalian_control(uint8_t rx_device_id)
    : Node("hardware_control_"+std::to_string(INPUT_DEVICE_ID)), rx_device_id_(rx_device_id)
{

    sensor_sub_2 = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_"+std::to_string(INPUT_DEVICE_ID),
        10,
        std::bind(&Shivalian_control::sensor_callback_2,
            this,
            std::placeholders::_1));
    
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 10);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(PUBLISH_RATE_MS), 
        std::bind(&Shivalian_control::publisher_position_callback, this));

    RCLCPP_INFO(this->get_logger(), "odometry node has also been started.");

}

void 
Shivalian_control::sensor_callback_2(
    const std_msgs::msg::Int16MultiArray::SharedPtr msg){   
    current = this->now();

    // 最低限：サイズチェック
    if (msg->data.size() < RX16NUM) {
        RCLCPP_WARN(this->get_logger(),
                    "serial_rx_%d: data too short (%zu)",
                    device_id_, msg->data.size());
        return;
    }

    if(!topic_received){
        
        point_Px = 0.0;//ノード起動時の座標を原点とする
        point_Py = 0.0;
        dx = 0.0;
        dy = 0.0;
        dx_r = 0.0;
        dy_r = 0.0;
        d_rad = 0.0;
        yaw = 0.0;
        last =this->now();
        topic_received = true;
        return;
    }

    dt = (current - last).seconds();

    // 以降、受信データを使った処理を記述

    int16_t ENC1 = msg->data[1];
    int16_t ENC2 = msg->data[2];
    int16_t ENC3 = msg->data[3];

    enc[0] = ENC1;
    enc[1] = ENC2;
    enc[2] = ENC3;

    for(int i=0; i<3; i++){
        diff[i] = enc[i] - last_enc[i];
        rps[i] = -diff[i] / (dt*cpr);
        last_enc[i] = enc[i];
        v[i] = ODOM_WHEEL_CIRC * rps[i];//各車輪のスカラーを算出(向きは半径ODOM_LR_DISTANCEの接線方向)
    }

    V[0] =Ma ({{v[0]},
                 {0}});//正面かつx軸正の方向を進行方向とする列ベクトル

    Ma ii = Ma ({{1},
                 {0}}); //単位ベクトル(x軸)

    V[1] = Ma (rot(120.0) * ii*v[1]); // θ=120°の回転行列をかけた方向が進行方向(行列で計算するとは言っていない)
    V[2] = Ma (rot(120.0 + 120.0) * ii*v[2]); // さらにθ=120°の回転行列をかけた方向が進行方向

    V_ = Ma ((V[0] + V[1] + V[2]) / (3.0*sin(opPI/3.0))); 
    v_ = (v[0] + v[1] + v[2]) / 3.0;//接線方向の速度の平均 
    d_rad = v_ / ODOM_LR_DISTANCE; //v=rωより、角速度ω=v/rで計算できる。r(ODOM_LR_DISTANCE)は設計されてないから分からない

    vx = V_.operator()(0,0);
    vy = V_.operator()(1,0);

    q_z = sin(yaw / 2.0);//クォータニオンのz成分
    q_w = cos(yaw / 2.0);//クォータニオンのw成分

    dx_r = vx * dt;//ロボットを原点とした基準での直交座標系の変位
    dy_r = vy * dt;

    dR_r = Ma ({{dx_r},
                {dy_r}}); //ロボットを原点とした基準での直交座標系の変位ベクトル

    dR = Ma (rot(yaw * 180.0 / opPI) * dR_r); //現在のロボットの初期方向からの傾き(yaw)をかけることで座標変換

    dx = dR.operator()(0,0);
    dy = dR.operator()(1,0);

    /*
    Ma dP_r = Ma ({{dx},
                   {dy}},
                   {d_rad*dt}); //ロボットを原点とした基準での直交座標系の変位ベクトル。z成分は角度の変化量d_rad*dt
    
    Ma rot3 = Ma ({{cos(yaw), -sin(yaw),0},
                   {sin(yaw), cos(yaw), 0},
                   { 0,        0,       1}}); // 3×3のyaw回転行列
    
    dP = rot3 * dR_r; //ロボットを原点とした基準での直交座標系の変位ベクトルに、現在のロボットの初期方向からの傾き(yaw)をかけることで座標変換
    
    point_Px += dP.operator()(0,0);
    point_Py += dP.operator()(1,0);
    yaw += dP.operator()(2,0);

    yaw =std::atan2(sin(yaw), cos(yaw));//atan2を通すことでyaw_の増長を防ぐ
    */


    point_Px += dx;//ロボットが起動した位置を原点とした現在位置
    point_Py += dy;

    yaw += d_rad * dt;
    yaw = atan2(sin(yaw), cos(yaw));//atan2を通すことでyaw_の増長を防ぐ

    last = current;

    // 受信データ処理ここまで
}

//publish
void Shivalian_control::publisher_position_callback()
{
    nav_msgs::msg::Odometry odom_msg;

    //sensor_callback_2で計算した位置と速度をodom_msgにセットしてpublishする

    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = point_Px;
    odom_msg.pose.pose.position.y = point_Py; 
    odom_msg.pose.pose.position.z = 0.0;  //z軸での計算はいたしません
    odom_msg.pose.pose.orientation.x =0.0;
    odom_msg.pose.pose.orientation.y =0.0;
    odom_msg.pose.pose.orientation.z =q_z;
    odom_msg.pose.pose.orientation.w =q_w;

    odom_msg.twist.twist.linear.x = vx;
    odom_msg.twist.twist.linear.y = vy;
    odom_msg.twist.twist.linear.z = 0.0;//z軸での計算は(ry
    odom_msg.twist.twist.angular.x = 0.0;//(Roll)=0  z=0ならこの2つは0
    odom_msg.twist.twist.angular.y = 0.0;//(Pitch)=0
    odom_msg.twist.twist.angular.z = d_rad;//(Yaw)まだ計算していない
    
    odom_pub_->publish(odom_msg);

    geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->get_clock()->now();
        tf.header.frame_id = "odom";
        tf.child_frame_id = "base_link";
        tf.transform.translation.x = point_Px;
        tf.transform.translation.y = point_Py;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = q_z;
        tf.transform.rotation.w = q_w;
        tf_broadcaster_->sendTransform(tf);
}

//回転行列を返す関数。引数は回転角度(度数法)
Ma Shivalian_control::rot(float degree){
    float rad = degree * opPI / 180.0;
    Ma A_rot = Ma ({{cos(rad), -sin(rad)},
                    {sin(rad), cos(rad)}}); // θ=degreeの回転行列
    return A_rot;
}

//右回転の回転行列を返す関数
Ma Shivalian_control::rotR(float degree){
    float rad = degree * opPI / 180.0;
    Ma A_rot = Ma ({{cos(rad), sin(rad)},
                    {-sin(rad), cos(rad)}});
    return A_rot;
}
