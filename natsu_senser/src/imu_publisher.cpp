// ESP32 (MPU9250) → ROS2 /imu publisher
// シリアル行フォーマット: "IMU:<roll>,<pitch>,<yaw>,<step_flag>"  (角度: 度)
// Publish: /imu (sensor_msgs/Imu), /imu/step_detected (std_msgs/Bool)

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstdio>
#include <string>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

class ImuPublisher : public rclcpp::Node {
public:
  ImuPublisher() : Node("imu_publisher") {
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB1");
    declare_parameter<int>("baud_rate", 115200);

    const std::string port = get_parameter("serial_port").as_string();
    const int baud         = get_parameter("baud_rate").as_int();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "シリアルポートを開けません: %s", port.c_str());
      throw std::runtime_error("serial open failed: " + port);
    }
    configure_serial(baud);

    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    step_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/step_detected", 10);

    // 5ms 間隔でポーリング (ESP32は 100Hz 出力)
    timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&ImuPublisher::read_and_publish, this));

    RCLCPP_INFO(get_logger(), "imu_publisher 起動: %s @%d baud", port.c_str(), baud);
  }

  ~ImuPublisher() {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void configure_serial(int baud) {
    struct termios tty{};
    tcgetattr(fd_, &tty);

    speed_t speed = B115200;
    if      (baud == 9600)   speed = B9600;
    else if (baud == 57600)  speed = B57600;
    else if (baud == 230400) speed = B230400;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void read_and_publish() {
    char buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf) - 1);
    if (n <= 0) return;

    buf[n] = '\0';
    line_buf_ += buf;

    size_t pos;
    while ((pos = line_buf_.find('\n')) != std::string::npos) {
      std::string line = line_buf_.substr(0, pos);
      // Windows改行 (\r\n) に対応
      if (!line.empty() && line.back() == '\r') line.pop_back();
      line_buf_ = line_buf_.substr(pos + 1);
      parse_line(line);
    }

    // バッファが肥大化しないよう上限を設ける
    if (line_buf_.size() > 512) line_buf_.clear();
  }

  void parse_line(const std::string &line) {
    if (line.rfind("IMU:", 0) != 0) return;

    float roll, pitch, yaw;
    int   step;
    if (std::sscanf(line.c_str() + 4, "%f,%f,%f,%d",
                    &roll, &pitch, &yaw, &step) != 4) return;

    // 度 → ラジアン
    const float r = roll  * static_cast<float>(M_PI) / 180.0f;
    const float p = pitch * static_cast<float>(M_PI) / 180.0f;
    const float y = yaw   * static_cast<float>(M_PI) / 180.0f;

    // ZYX オイラー角 → クォータニオン
    const float cy = std::cos(y * 0.5f), sy = std::sin(y * 0.5f);
    const float cp = std::cos(p * 0.5f), sp = std::sin(p * 0.5f);
    const float cr = std::cos(r * 0.5f), sr = std::sin(r * 0.5f);

    sensor_msgs::msg::Imu imu_msg;
    imu_msg.header.stamp    = now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.orientation.w = cr*cp*cy + sr*sp*sy;
    imu_msg.orientation.x = sr*cp*cy - cr*sp*sy;
    imu_msg.orientation.y = cr*sp*cy + sr*cp*sy;
    imu_msg.orientation.z = cr*cp*sy - sr*sp*cy;

    // roll/pitch は加速度補正あり (比較的信頼), yaw は積分のみ
    imu_msg.orientation_covariance[0] = 0.01;   // roll
    imu_msg.orientation_covariance[4] = 0.01;   // pitch
    imu_msg.orientation_covariance[8] = 0.05;   // yaw

    // raw 角速度・加速度は送信していないため未提供を示す (-1)
    imu_msg.angular_velocity_covariance[0]    = -1.0;
    imu_msg.linear_acceleration_covariance[0] = -1.0;

    imu_pub_->publish(imu_msg);

    std_msgs::msg::Bool step_msg;
    step_msg.data = (step != 0);
    step_pub_->publish(step_msg);
  }

  int fd_{-1};
  std::string line_buf_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   step_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImuPublisher>());
  rclcpp::shutdown();
  return 0;
}
