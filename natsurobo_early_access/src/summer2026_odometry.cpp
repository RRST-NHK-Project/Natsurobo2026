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
    
    // timer_callbackを呼び出すタイマーを作成
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
        last =this->now();
        topic_received = true;
        return;
    }

    // 以降、受信データを使った処理を記述

    int16_t ENC_1 = msg->data[0];
    int16_t ENC_2 = msg->data[1];
    int16_t ENC_3 = msg->data[2];

    enc[0] = ENC_1;
    enc[1] = ENC_2;
    enc[2] = ENC_3;

    for(int i=0; i<3; i++){
        diff[i] = enc[i] - last_enc[i];
        rps[i] = -diff[i] / (dt*cpr);
        last_enc[i] = enc[i];
    }
    V[0] = ODOM_WHEEL_CIRC*rps[0];//正面かつx軸正を進行方向とする
    V[1] = ODOM_WHEEL_CIRC*rps[1];//θ=120°の回転行列をかけた方向が進行方向(行列で計算するとは言っていない)
    V[2] = ODOM_WHEEL_CIRC*rps[2];//さらにθ=120°の回転行列をかけた方向が進行方向

    V_ = (V[0] + V[1] + V[2]) / 3.0;
    
    dt = PUBLISH_RATE_MS / 1000.0;
    
    
    Vx_ = (V[0]*std::cos(opPI/2.0) + V[1]*-std::sin(opPI/6.0) + V[2]*-std::cos(opPI/6.0)) / 3.0;
    Vy_ = (V[0]*std::sin(opPI/2.0) + V[1]*-std::cos(opPI/6.0) + V[2]*std::sin(opPI/6.0)) / 3.0;

    q_rad = atan2(Vy_, Vx_); 
    q_z = sin(q_rad / 2.0);
    q_w = cos(q_rad / 2.0);

    point_Px += Vx_ * dt;
    point_Py += Vy_ * dt;

    //v=rωより、角速度ω=v/rで計算できる。r(ODOM_RADIUS)は設計されてないから分からない

    d_rad = V_ / ODOM_RADIUS; 

    last = current;

    // 受信データ処理ここまで
}

//publish
void Shivalian_control::publisher_position_callback()
{
    nav_msgs::msg::Odometry odom_msg;

    //受信したrpsを元にオドメトリの値を計算する処理をここに追加

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

    odom_msg.twist.twist.linear.x = Vx_;
    odom_msg.twist.twist.linear.y = Vy_;
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
