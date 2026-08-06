// WT901C (WitMotion) → ROS2 /imu publisher
// シリアルから 11バイトパケット [0x55][種別][データ8バイト][チェックサム] を直接パース
//   0x51: 加速度 (±16g) / 0x52: 角速度 (±2000deg/s) / 0x53: 姿勢角 (±180deg)
// 起動時に6軸アルゴリズム（地磁気オフ）を設定コマンドで書き込む
// Publish: /imu (sensor_msgs/Imu), /imu/step_detected (std_msgs/Bool)
//地磁気は使いません！

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/bool.hpp"

class Wt901cPublisher : public rclcpp::Node {
public:
  Wt901cPublisher() : Node("wt901c_publisher") {
    declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    declare_parameter<int>("baud_rate", 9600);
    declare_parameter<std::string>("frame_id", "imu_link");
    declare_parameter<bool>("configure_6axis", true);   // 起動時に地磁気オフ(6軸)を書き込む
    declare_parameter<int>("set_output_rate_hz", 0);    // 0=変更しない (10/20/50/100/200)
    declare_parameter<double>("step_accel_thresh", 7.0); // 段差検知: 水平加速度 |ax|+|ay| [m/s^2]
    declare_parameter<int>("step_cooldown_ms", 500);

    const std::string port = get_parameter("serial_port").as_string();
    const int baud         = get_parameter("baud_rate").as_int();
    frame_id_              = get_parameter("frame_id").as_string();
    step_thresh_           = get_parameter("step_accel_thresh").as_double();
    step_cooldown_ms_      = get_parameter("step_cooldown_ms").as_int();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
      RCLCPP_FATAL(get_logger(), "シリアルポートを開けません: %s", port.c_str());
      throw std::runtime_error("serial open failed: " + port);
    }
    configure_serial(baud);

    if (get_parameter("configure_6axis").as_bool()) {
      configure_sensor(baud);
    }

    imu_pub_  = create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
    step_pub_ = create_publisher<std_msgs::msg::Bool>("/imu/step_detected", 10);

    timer_ = create_wall_timer(
      std::chrono::milliseconds(5),
      std::bind(&Wt901cPublisher::read_serial, this));

    RCLCPP_INFO(get_logger(), "wt901c_publisher 起動: %s @%d baud", port.c_str(), baud);
  }

  ~Wt901cPublisher() {
    if (fd_ >= 0) ::close(fd_);
  }

private:
  void configure_serial(int baud) {
    struct termios tty{};
    tcgetattr(fd_, &tty);

    speed_t speed = B9600;
    if      (baud == 57600)  speed = B57600;
    else if (baud == 115200) speed = B115200;
    else if (baud == 230400) speed = B230400;

    cfsetispeed(&tty, speed);
    cfsetospeed(&tty, speed);
    cfmakeraw(&tty);
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;
    tcsetattr(fd_, TCSANOW, &tty);
  }

  void send_cmd(const uint8_t cmd[5]) {
    if (::write(fd_, cmd, 5) != 5) {
      RCLCPP_WARN(get_logger(), "設定コマンドの送信に失敗しました (reg=0x%02X)", cmd[2]);
    }
    usleep(100 * 1000); // レジスタ書き込みの反映待ち
  }

  // 地磁気オフ(6軸アルゴリズム) + 出力内容を加速度/角速度/姿勢角のみに設定
  void configure_sensor(int baud) {
    const uint8_t unlock[5] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    const uint8_t axis6[5]  = {0xFF, 0xAA, 0x24, 0x01, 0x00}; // 0x01=6軸, 0x00=9軸
    const uint8_t rsw[5]    = {0xFF, 0xAA, 0x02, 0x0E, 0x00}; // 0x0E=加速度+角速度+姿勢角 (地磁気パケット停止)
    const uint8_t save[5]   = {0xFF, 0xAA, 0x00, 0x00, 0x00};

    send_cmd(unlock);
    send_cmd(axis6);
    send_cmd(rsw);

    const int rate_hz = get_parameter("set_output_rate_hz").as_int();
    if (rate_hz > 0) {
      uint8_t code = 0;
      if      (rate_hz == 10)  code = 0x06;
      else if (rate_hz == 20)  code = 0x07;
      else if (rate_hz == 50)  code = 0x08;
      else if (rate_hz == 100) code = 0x09;
      else if (rate_hz == 200) code = 0x0B;

      if (code == 0) {
        RCLCPP_WARN(get_logger(), "未対応の出力レート %dHz (10/20/50/100/200のみ)", rate_hz);
      } else if (rate_hz * 33 > baud / 10) {
        // 1周期 = 3パケット×11バイト。ボーレートが足りないと欠落する
        RCLCPP_WARN(get_logger(),
                    "%dHz は %dbaud では帯域不足のため設定しません (115200baud を推奨)",
                    rate_hz, baud);
      } else {
        const uint8_t rrate[5] = {0xFF, 0xAA, 0x03, code, 0x00};
        send_cmd(rrate);
        RCLCPP_INFO(get_logger(), "出力レートを %dHz に設定", rate_hz);
      }
    }

    send_cmd(save);
    RCLCPP_INFO(get_logger(), "WT901C を6軸モード(地磁気オフ)に設定しました");
  }

  void read_serial() {
    uint8_t buf[256];
    ssize_t n = ::read(fd_, buf, sizeof(buf));
    if (n <= 0) return;

    rx_.insert(rx_.end(), buf, buf + n);

    while (rx_.size() >= 11) {
      if (rx_[0] != 0x55) {
        rx_.erase(rx_.begin());
        continue;
      }
      uint8_t sum = 0;
      for (int i = 0; i < 10; i++) sum += rx_[i];
      if (sum != rx_[10]) {
        rx_.erase(rx_.begin()); // ヘッダ誤検出: 1バイトずらして再同期
        continue;
      }
      parse_packet(&rx_[0]);
      rx_.erase(rx_.begin(), rx_.begin() + 11);
    }
  }

  static int16_t s16(const uint8_t *p) {
    return static_cast<int16_t>(p[0] | (p[1] << 8));
  }

  void parse_packet(const uint8_t *p) {
    const uint8_t *d = p + 2;
    switch (p[1]) {
      case 0x51: { // 加速度 (±16g) → m/s^2
        ax_ = s16(d + 0) / 32768.0 * 16.0 * 9.8;
        ay_ = s16(d + 2) / 32768.0 * 16.0 * 9.8;
        az_ = s16(d + 4) / 32768.0 * 16.0 * 9.8;
        detect_step();
        break;
      }
      case 0x52: { // 角速度 (±2000deg/s) → rad/s
        gx_ = s16(d + 0) / 32768.0 * 2000.0 * M_PI / 180.0;
        gy_ = s16(d + 2) / 32768.0 * 2000.0 * M_PI / 180.0;
        gz_ = s16(d + 4) / 32768.0 * 2000.0 * M_PI / 180.0;
        break;
      }
      case 0x53: { // 姿勢角 (±180deg) → 1周期の最後に来るのでここでpublish
        const double roll  = s16(d + 0) / 32768.0 * M_PI;
        const double pitch = s16(d + 2) / 32768.0 * M_PI;
        const double yaw   = s16(d + 4) / 32768.0 * M_PI;
        publish_imu(roll, pitch, yaw);
        break;
      }
      default: // 0x54(地磁気)等はRSW設定で停止済みだが、来ても無視
        break;
    }
  }

  // 段差乗り越え時の水平加速度スパイクを検知 (mpu9250_esp32.ino から移植)
  void detect_step() {
    const bool shake = (std::abs(ax_) + std::abs(ay_)) > step_thresh_;
    const auto now_ms = now().nanoseconds() / 1000000;

    if (shake) {
      if (!step_active_) RCLCPP_INFO(get_logger(), "段差検知: |ax|+|ay|=%.1f m/s^2", std::abs(ax_) + std::abs(ay_));
      step_active_ = true;
      step_time_ms_ = now_ms;
    } else if (step_active_ && (now_ms - step_time_ms_) > step_cooldown_ms_) {
      step_active_ = false;
    }
  }

  void publish_imu(double roll, double pitch, double yaw) {
    const double cy = std::cos(yaw * 0.5),   sy = std::sin(yaw * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cr = std::cos(roll * 0.5),  sr = std::sin(roll * 0.5);

    sensor_msgs::msg::Imu msg;
    msg.header.stamp    = now();
    msg.header.frame_id = frame_id_;

    msg.orientation.w = cr*cp*cy + sr*sp*sy;
    msg.orientation.x = sr*cp*cy - cr*sp*sy;
    msg.orientation.y = cr*sp*cy + sr*cp*sy;
    msg.orientation.z = cr*cp*sy - sr*sp*cy;

    // roll/pitch はセンサー内カルマン融合済み。yaw は6軸モードではジャイロ積分のみ
    msg.orientation_covariance[0] = 0.01;
    msg.orientation_covariance[4] = 0.01;
    msg.orientation_covariance[8] = 0.05;

    msg.angular_velocity.x = gx_;
    msg.angular_velocity.y = gy_;
    msg.angular_velocity.z = gz_;
    msg.angular_velocity_covariance[0] = 0.001;
    msg.angular_velocity_covariance[4] = 0.001;
    msg.angular_velocity_covariance[8] = 0.001;

    msg.linear_acceleration.x = ax_;
    msg.linear_acceleration.y = ay_;
    msg.linear_acceleration.z = az_;
    msg.linear_acceleration_covariance[0] = 0.05;
    msg.linear_acceleration_covariance[4] = 0.05;
    msg.linear_acceleration_covariance[8] = 0.05;

    imu_pub_->publish(msg);

    std_msgs::msg::Bool step_msg;
    step_msg.data = step_active_;
    step_pub_->publish(step_msg);
  }

  int fd_{-1};
  std::string frame_id_;
  std::vector<uint8_t> rx_;

  double ax_{0}, ay_{0}, az_{0};
  double gx_{0}, gy_{0}, gz_{0};

  double step_thresh_{7.0};
  int step_cooldown_ms_{500};
  bool step_active_{false};
  int64_t step_time_ms_{0};

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   step_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Wt901cPublisher>());
  rclcpp::shutdown();
  return 0;
}
