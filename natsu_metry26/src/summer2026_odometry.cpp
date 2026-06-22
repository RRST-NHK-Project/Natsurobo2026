#include "summer2026_odometry.hpp"

/*
3輪オドメトリの回転をエンコーダから受け取ってオドメトリの自己位置を計算し、odomとtfを送信する（但し現状では無意味）。
Copyright (c) 2025 RRST-NHK-Project. All rights reserved.
*/
Shivalian_control::Shivalian_control(uint8_t rx_device_id)
    : Node("guess_position_26" + std::to_string(INPUT_DEVICE_ID)), rx_device_id_(rx_device_id)
{

    sensor_sub_2 = this->create_subscription<std_msgs::msg::Int16MultiArray>(
        "serial_rx_" + std::to_string(INPUT_DEVICE_ID),
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

void Shivalian_control::sensor_callback_2(
    const std_msgs::msg::Int16MultiArray::SharedPtr msg)
{
    current = this->now();

    // 最低限：サイズチェック
    if (msg->data.size() < RX16NUM)
    {
        RCLCPP_WARN(this->get_logger(),
                    "serial_rx_%d: data too short (%zu)",
                    device_id_, msg->data.size());
        return;
    }

    if (!topic_received)
    {
        {
            std::lock_guard<std::mutex> lock(read_only); // 他の関数からの書き換えを防止するためのロック。
            point_Px = 0.0;                              // ノード起動時の座標を原点とする
            point_Py = 0.0;
            dx = 0.0;
            dy = 0.0;
            dx_r = 0.0;
            dy_r = 0.0;
            d_rad = 0.0;
            yaw = 0.0;
            last = this->now();
            topic_received = true;
        }
        return;
    }

    {
        std::lock_guard<std::mutex> lock(read_only); // 他の関数からの書き換えを防止するためのロック。

        dt = (current - last).seconds();
    }

    // 以降、受信データを使った処理を記述

    ENC1 = msg->data[0];
    ENC2 = msg->data[1];
    ENC3 = msg->data[2];

    enc[0] = ENC1;
    enc[1] = ENC2;
    enc[2] = ENC3;

    for (int i = 0; i < 3; i++)
    {
        diff[i] = enc[i] - last_enc[i];
        rps[i] = -diff[i] / (dt * cpr); // Mark Ⅱが-diff[i]だっただけでMark Ⅲがどうなるかは不明。
        last_enc[i] = enc[i];
        v[i] = ODOM_WHEEL_CIRC * rps[i]; // 各車輪のスカラーを算出(向きは半径ODOM_LR_DISTANCEの接線方向)
    }

    V_wheel = matrix({{v[0]},
                      {v[1]},
                      {v[2]}}); // 各車輪の速度をベクトル化

    FK_inv = FK.inv();

    V_r = FK_inv * V_wheel; // タイヤの速度ベクトルから、ロボットを原点とした基準での直交座標系の速度ベクトルへ

    vx_r = V_r.operator()(0, 0); //(i-1,j-1にあたる成分を取り出す)
    vy_r = V_r.operator()(1, 0);
    d_rad = V_r.operator()(2, 0);

    dx_r = vx_r * dt;
    dy_r = vy_r * dt;

    dR_r = matrix({{dx_r},
                   {dy_r},
                   {d_rad * dt}}); // ロボットを原点とした基準での直交座標系の変位ベクトル。z成分は角度の変化量d_rad*dt

    R = matrix({{cos(yaw), -sin(yaw), 0},
                {sin(yaw), cos(yaw), 0},
                {0, 0, 1}}); // 3×3のyaw回転行列(動力学で出てくる運動座標系A-ξηから固定座標系O-xyへの変換行列)

    dR = R * dR_r; // ロボットを原点とした基準での直交座標系の変位ベクトルに、現在のロボットの初期方向からの傾き(yaw)をかけることで座標変換

    dx = dR.operator()(0, 0);
    dy = dR.operator()(1, 0);
    d_yaw = dR.operator()(2, 0);

    {
        std::lock_guard<std::mutex> lock(read_only); // 他の関数からの書き換えを防止するためのロック。

        point_Px += dx; // ロボットが起動した位置を原点とした現在位置
        point_Py += dy;

        yaw += d_yaw;
        yaw = atan2(sin(yaw), cos(yaw)); // atan2を通すことでyaw_の増長を防ぐ

        q_z = sin(yaw / 2.0);
        q_w = cos(yaw / 2.0);

        last = current;
    }

    // 受信データ処理ここまで
}

// publish
void Shivalian_control::publisher_position_callback()
{
    nav_msgs::msg::Odometry odom_msg;

    std::lock_guard<std::mutex> lock(read_only);

    // sensor_callback_2で計算した位置と速度をodom_msgにセットしてpublishする

    odom_msg.header.stamp = this->get_clock()->now();
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    odom_msg.pose.pose.position.x = point_Px;
    odom_msg.pose.pose.position.y = point_Py;
    odom_msg.pose.pose.position.z = 0.0; // z軸での計算はいたしません
    odom_msg.pose.pose.orientation.x = 0.0;
    odom_msg.pose.pose.orientation.y = 0.0;
    odom_msg.pose.pose.orientation.z = q_z;
    odom_msg.pose.pose.orientation.w = q_w;

    odom_msg.twist.twist.linear.x = vx_r;
    odom_msg.twist.twist.linear.y = vy_r;
    odom_msg.twist.twist.linear.z = 0.0;    // z軸での計算は(ry
    odom_msg.twist.twist.angular.x = 0.0;   //(Roll)=0  z=0ならこの2つは0
    odom_msg.twist.twist.angular.y = 0.0;   //(Pitch)=0
    odom_msg.twist.twist.angular.z = d_rad; //(Yaw)

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
    RCLCPP_INFO(this->get_logger(), "Position: (%.2f, %.2f)(m), Yaw: %.2f(rad), vx_r,vy_r: (%.2f, %.2f)(m/s), d_rad: %.2f(rad), dt: %.4f(s), Encoders: (%d, %d, %d), Wheel Velocities: (%.2f, %.2f, %.2f)(m/s), q_z: %.2f(rad), q_w: %.2f(rad)",
                point_Px, point_Py, yaw, vx_r, vy_r, d_rad, dt, ENC1, ENC2, ENC3, v[0], v[1], v[2], q_z, q_w);
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Shivalian_control>(INPUT_DEVICE_ID);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}