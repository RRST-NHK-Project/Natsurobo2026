/*
 * natsu_climb_seq_node.cpp
 *
 * 夏ロボ2026 「昇降まで」実行ノード(executor)
 *
 * 役割分担:
 *   natsu_auto_node = 状態遷移の記述・管理(司令塔)。/auto/state を出すだけ。
 *   このノード       = /auto/state を見て、各プログラムの出入り口へ実際の司令を出し、
 *                     フェーズ完了を判定して /auto/phase_done を司令塔へ返す。
 *
 * 対象フェーズ(/auto/state の値):
 *   "DETECT_STEP" : LiDAR /wall_detection/distance で段まで詰める → /cmd_vel(前進)
 *   "ALIGN"       : 開始時に LiDAR /wall_detection/angle で目標yawを決め、以後 IMU /imu の
 *                   yaw をFBに段へ平行になるまで旋回 → /cmd_vel(旋回)
 *   "CLIMB"       : 段差超えシーケンス。omni前進(/cmd_vel)とシリンダ順次格納(/climb/cylinder)を
 *                   時間+距離(/odom)でシーケンスし、IMU /imu の衝撃で各段の成功を確認する。
 *                   シーケンス: [シリンダ上げ]→[前進(前輪を段へ)]→[前輪格納]→[前進(後輪を段へ)]
 *                              →[後輪格納]  (衝撃3回: 上げ / 前輪乗り / 後輪乗り を期待)
 *
 * 各フェーズ完了(またはタイムアウト保険)で /auto/phase_done にフェーズ名を1回publishする。
 *
 * ★遷移(=完了判定)は「その瞬間の1サンプル」では確定しない:
 *   - しきい値を confirm_frames 回連続で満たすこと(デバウンス)
 *   - かつフェーズに入ってから min_state_dwell 秒経過していること
 *
 * subscribe:
 *   /auto/state              [std_msgs/String]       現在状態(司令塔から)
 *   /wall_detection/distance [std_msgs/Float64]      段までの距離[m]
 *   /wall_detection/angle    [std_msgs/Float64]      段の偏角[rad]
 *   /imu                     [sensor_msgs/Imu]       姿勢+加速度
 *   /odom                    [nav_msgs/Odometry]     自己位置(登坂中の前進距離用)
 *
 * publish:
 *   /cmd_vel        [geometry_msgs/Twist]        走行指令(→zakiomni。arbitration=auto時のみ有効)
 *   /climb/cylinder [std_msgs/Int16MultiArray]   シリンダ状態[front, rear](→unaginobori)
 *   /auto/phase_done[std_msgs/String]            フェーズ完了通知(→natsu_auto_node)
 *
 * 注意: /auto/arbitration は natsu_auto_node が発行する(このノードは出さない)。
 */

#include <cmath>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/int16_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"

// CLIMBフェーズ内部の段差超えサブシーケンス
enum class ClimbStep {
    RAISE,        // 両シリンダ上げ(衝撃#1: 上げ)
    FWD_FRONT,    // 前進して前輪を段に乗せる(衝撃#2: 前輪乗り)
    STOW_FRONT,   // 前輪シリンダ格納
    FWD_REAR,     // 前進して後輪を段に乗せる(衝撃#3: 後輪乗り)
    STOW_REAR,    // 後輪シリンダ格納
    FINISH        // 完了
};

class NatsuClimbSeqNode : public rclcpp::Node
{
public:
    NatsuClimbSeqNode() : Node("natsu_climb_seq_node")
    {
        // ── 共通(デバウンス/滞在/鮮度) ─────────────────
        declare_parameter<int>("confirm_frames", 3);
        declare_parameter<double>("min_state_dwell", 0.3);
        declare_parameter<double>("sensor_timeout", 0.5);

        // ── DETECT_STEP ────────────────────────────────
        declare_parameter<double>("target_distance", 0.25);
        declare_parameter<double>("approach_v", 0.30);
        declare_parameter<double>("approach_kp", 1.0);
        declare_parameter<double>("approach_timeout", 6.0);

        // ── ALIGN ──────────────────────────────────────
        declare_parameter<double>("align_wall_angle_sign", 1.0);
        declare_parameter<double>("align_angle_tol", 0.05);
        declare_parameter<double>("align_kp", 1.5);
        declare_parameter<double>("align_omega_max", 1.0);
        declare_parameter<double>("align_timeout", 8.0);

        // ── CLIMB: 段差超えシーケンス ───────────────────
        // 上げ後、動き出す前の保持 [s]
        declare_parameter<double>("climb_raise_hold", 1.0);
        // 前進(前輪を段へ)の目標距離[m] / 速度[m/s] / 打ち切り[s]
        declare_parameter<double>("climb_fwd_front_dist", 0.30);
        declare_parameter<double>("climb_fwd_front_v", 0.20);
        declare_parameter<double>("climb_fwd_front_timeout", 5.0);
        // 前輪格納の保持 [s]
        declare_parameter<double>("climb_stow_front_hold", 0.8);
        // 前進(後輪を段へ)の目標距離[m] / 速度[m/s] / 打ち切り[s]
        declare_parameter<double>("climb_fwd_rear_dist", 0.30);
        declare_parameter<double>("climb_fwd_rear_v", 0.20);
        declare_parameter<double>("climb_fwd_rear_timeout", 5.0);
        // 後輪格納の保持 [s]
        declare_parameter<double>("climb_stow_rear_hold", 0.8);

        // ── IMU衝撃検知 ─────────────────────────────────
        // 加速度の大きさの重力からのずれ |‖a‖-g| がこれを超えたら衝撃 [m/s^2]
        declare_parameter<double>("shock_accel_thresh", 7.0);
        // 衝撃の多重カウント防止クールダウン [s]
        declare_parameter<double>("shock_cooldown", 0.4);
        // true: 前進フェーズは期待する衝撃を検知するまで(距離到達でも)完了扱いにしない。
        //       false: 時間+距離だけで進め、衝撃は成功確認のログのみ(初期はこちら推奨)
        declare_parameter<bool>("climb_require_shock", false);

        declare_parameter<double>("rate_hz", 20.0);

        load_params();

        // ── Publisher ───────────────────────────────
        cmd_vel_pub_    = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        cylinder_pub_   = create_publisher<std_msgs::msg::Int16MultiArray>("/climb/cylinder", 10);
        phase_done_pub_ = create_publisher<std_msgs::msg::String>("/auto/phase_done", 10);

        // ── Subscriber ──────────────────────────────
        state_sub_ = create_subscription<std_msgs::msg::String>(
            "/auto/state", 10,
            [this](std_msgs::msg::String::SharedPtr m){ on_state(m->data); });

        dist_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/distance", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){
                wall_dist_ = m->data; wall_dist_stamp_ = now(); });

        angle_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){
                wall_angle_ = m->data; wall_angle_stamp_ = now(); });

        imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
            "/imu", 50,
            [this](sensor_msgs::msg::Imu::SharedPtr m){ on_imu(*m); });

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            [this](nav_msgs::msg::Odometry::SharedPtr m){
                odom_x_ = m->pose.pose.position.x;
                odom_y_ = m->pose.pose.position.y;
                odom_stamp_ = now(); });

        const int ms = static_cast<int>(1000.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(ms),
            std::bind(&NatsuClimbSeqNode::loop, this));

        RCLCPP_INFO(get_logger(), "natsu_climb_seq_node started (executor).");
    }

private:
    void load_params()
    {
        confirm_frames_  = get_parameter("confirm_frames").as_int();
        min_state_dwell_ = get_parameter("min_state_dwell").as_double();
        sensor_timeout_  = get_parameter("sensor_timeout").as_double();

        target_distance_ = get_parameter("target_distance").as_double();
        approach_v_      = get_parameter("approach_v").as_double();
        approach_kp_     = get_parameter("approach_kp").as_double();
        approach_timeout_= get_parameter("approach_timeout").as_double();

        align_sign_      = get_parameter("align_wall_angle_sign").as_double();
        align_tol_       = get_parameter("align_angle_tol").as_double();
        align_kp_        = get_parameter("align_kp").as_double();
        align_omega_max_ = get_parameter("align_omega_max").as_double();
        align_timeout_   = get_parameter("align_timeout").as_double();

        raise_hold_       = get_parameter("climb_raise_hold").as_double();
        fwd_front_dist_   = get_parameter("climb_fwd_front_dist").as_double();
        fwd_front_v_      = get_parameter("climb_fwd_front_v").as_double();
        fwd_front_timeout_= get_parameter("climb_fwd_front_timeout").as_double();
        stow_front_hold_  = get_parameter("climb_stow_front_hold").as_double();
        fwd_rear_dist_    = get_parameter("climb_fwd_rear_dist").as_double();
        fwd_rear_v_       = get_parameter("climb_fwd_rear_v").as_double();
        fwd_rear_timeout_ = get_parameter("climb_fwd_rear_timeout").as_double();
        stow_rear_hold_   = get_parameter("climb_stow_rear_hold").as_double();

        shock_thresh_    = get_parameter("shock_accel_thresh").as_double();
        shock_cooldown_  = get_parameter("shock_cooldown").as_double();
        require_shock_   = get_parameter("climb_require_shock").as_bool();

        rate_hz_ = get_parameter("rate_hz").as_double();
    }

    // ── 司令塔からの状態受信 ────────────────────
    void on_state(const std::string& s)
    {
        if (s == phase_) return;
        phase_ = s;
        phase_entered_ = now();
        reach_count_ = 0;
        align_count_ = 0;
        align_target_set_ = false;
        done_emitted_ = false;
        // CLIMBに入ったらサブシーケンスを頭から
        climb_step_ = ClimbStep::RAISE;
        climb_step_entered_ = now();
        climb_cmd_sent_ = false;
        shock_seen_this_step_ = false;
        RCLCPP_INFO(get_logger(), "phase → %s", phase_.c_str());
    }

    void on_imu(const sensor_msgs::msg::Imu& m)
    {
        const double x = m.orientation.x, y = m.orientation.y;
        const double z = m.orientation.z, w = m.orientation.w;

        double sinp = 2.0 * (w * y - z * x);
        sinp = std::clamp(sinp, -1.0, 1.0);
        imu_pitch_ = std::asin(sinp);

        const double siny = 2.0 * (w * z + x * y);
        const double cosy = 1.0 - 2.0 * (y * y + z * z);
        imu_yaw_ = std::atan2(siny, cosy);

        imu_stamp_ = now();

        // 衝撃検知: 加速度の大きさが重力からずれたスパイクを立ち上がりで数える
        const double ax = m.linear_acceleration.x;
        const double ay = m.linear_acceleration.y;
        const double az = m.linear_acceleration.z;
        const double dev = std::abs(std::sqrt(ax*ax + ay*ay + az*az) - 9.8);
        const bool over = dev > shock_thresh_;
        if (over && !shock_prev_over_ &&
            (now() - last_shock_time_).seconds() > shock_cooldown_) {
            last_shock_time_ = now();
            ++shock_total_;
            shock_seen_this_step_ = true;   // 現サブステップ中に衝撃あり
            RCLCPP_INFO(get_logger(), "衝撃検知 #%d (|a|-g=%.1f)", shock_total_, dev);
        }
        shock_prev_over_ = over;
    }

    // ── メインループ ────────────────────────────
    void loop()
    {
        if (done_emitted_) return;
        if      (phase_ == "DETECT_STEP") do_approach();
        else if (phase_ == "ALIGN")       do_align();
        else if (phase_ == "CLIMB")       do_climb();
    }

    double elapsed() const { return (now() - phase_entered_).seconds(); }

    // ── DETECT_STEP ────────────────────────────────
    void do_approach()
    {
        if (elapsed() > approach_timeout_) {
            RCLCPP_WARN(get_logger(), "接近を打ち切り (%.1fs経過)", elapsed());
            emit_done("DETECT_STEP");
            return;
        }
        if (!wall_dist_fresh()) { stop_robot(); reach_count_ = 0; return; }

        if (wall_dist_ <= target_distance_) ++reach_count_; else reach_count_ = 0;
        if (reach_count_ >= confirm_frames_ && elapsed() > min_state_dwell_) {
            RCLCPP_INFO(get_logger(), "段の前に到達: dist=%.2fm", wall_dist_);
            emit_done("DETECT_STEP");
            return;
        }
        const double err = wall_dist_ - target_distance_;
        geometry_msgs::msg::Twist t;
        t.linear.x = std::clamp(approach_kp_ * err, 0.0, approach_v_);
        cmd_vel_pub_->publish(t);
    }

    // ── ALIGN ──────────────────────────────────────
    void do_align()
    {
        if (elapsed() > align_timeout_) {
            RCLCPP_WARN(get_logger(), "平行旋回を打ち切り (%.1fs経過)", elapsed());
            emit_done("ALIGN");
            return;
        }
        if (!imu_fresh()) { stop_robot(); align_count_ = 0; return; }

        if (!align_target_set_) {
            if (!wall_angle_fresh()) { stop_robot(); return; }
            align_target_yaw_ = wrap_pi(imu_yaw_ + align_sign_ * wall_angle_);
            align_target_set_ = true;
            RCLCPP_INFO(get_logger(), "平行目標yaw=%.3f (wall_angle=%.3f)",
                        align_target_yaw_, wall_angle_);
        }
        const double yaw_err = wrap_pi(align_target_yaw_ - imu_yaw_);
        if (std::abs(yaw_err) < align_tol_) ++align_count_; else align_count_ = 0;
        if (align_count_ >= confirm_frames_ && elapsed() > min_state_dwell_) {
            RCLCPP_INFO(get_logger(), "平行完了: yaw_err=%.3f", yaw_err);
            emit_done("ALIGN");
            return;
        }
        geometry_msgs::msg::Twist t;
        t.angular.z = std::clamp(align_kp_ * yaw_err, -align_omega_max_, align_omega_max_);
        cmd_vel_pub_->publish(t);
    }

    // ── CLIMB: 段差超えサブシーケンス ───────────────
    void do_climb()
    {
        switch (climb_step_) {
            case ClimbStep::RAISE:      climb_raise();      break;
            case ClimbStep::FWD_FRONT:  climb_fwd(fwd_front_dist_, fwd_front_v_,
                                                  fwd_front_timeout_, ClimbStep::STOW_FRONT,
                                                  "前輪を段へ前進"); break;
            case ClimbStep::STOW_FRONT: climb_stow(0, 1, stow_front_hold_,
                                                   ClimbStep::FWD_REAR, "前輪格納"); break;
            case ClimbStep::FWD_REAR:   climb_fwd(fwd_rear_dist_, fwd_rear_v_,
                                                  fwd_rear_timeout_, ClimbStep::STOW_REAR,
                                                  "後輪を段へ前進"); break;
            case ClimbStep::STOW_REAR:  climb_stow(0, 0, stow_rear_hold_,
                                                   ClimbStep::FINISH, "後輪格納"); break;
            case ClimbStep::FINISH:
                RCLCPP_INFO(get_logger(), "登坂シーケンス完了(衝撃 %d回検知)", shock_total_);
                emit_done("CLIMB");
                break;
        }
    }

    double step_elapsed() const { return (now() - climb_step_entered_).seconds(); }

    void enter_climb_step(ClimbStep next)
    {
        climb_step_ = next;
        climb_step_entered_ = now();
        climb_cmd_sent_ = false;
        shock_seen_this_step_ = false;
    }

    // 両シリンダ上げ → raise_hold 保持(衝撃#1を期待)
    void climb_raise()
    {
        stop_robot();
        if (!climb_cmd_sent_) {
            publish_cylinder(1, 1);
            climb_cmd_sent_ = true;
            RCLCPP_INFO(get_logger(), "シリンダ上げ (raise_hold=%.1fs)", raise_hold_);
            return;
        }
        if (step_elapsed() > raise_hold_) {
            if (!shock_seen_this_step_)
                RCLCPP_WARN(get_logger(), "上げの衝撃#1が未検知(そのまま進む)");
            enter_climb_step(ClimbStep::FWD_FRONT);
        }
    }

    // omniで前進。距離到達 or 打ち切りで次へ(衝撃#2/#3を期待)。
    void climb_fwd(double dist, double v, double timeout, ClimbStep next, const char* tag)
    {
        if (!climb_cmd_sent_) {                 // 前進開始点の自己位置を記録
            fwd_start_x_ = odom_x_; fwd_start_y_ = odom_y_;
            climb_cmd_sent_ = true;
            RCLCPP_INFO(get_logger(), "%s: 目標%.2fm v=%.2f", tag, dist, v);
        }
        const double traveled = odom_fresh()
            ? std::hypot(odom_x_ - fwd_start_x_, odom_y_ - fwd_start_y_) : 0.0;
        const bool dist_ok  = odom_fresh() && traveled >= dist;
        const bool time_out = step_elapsed() > timeout;
        // 衝撃を必須にする設定なら、距離到達でも衝撃が来るまで待つ(打ち切りは無条件で抜ける)
        const bool shock_ok = !require_shock_ || shock_seen_this_step_;

        if ((dist_ok && shock_ok) || time_out) {
            stop_robot();
            if (time_out)
                RCLCPP_WARN(get_logger(), "%s 打ち切り(%.1fs, 進行%.2fm)", tag, step_elapsed(), traveled);
            else
                RCLCPP_INFO(get_logger(), "%s 完了(進行%.2fm, 衝撃%s)", tag, traveled,
                            shock_seen_this_step_ ? "有" : "無");
            enter_climb_step(next);
            return;
        }
        geometry_msgs::msg::Twist t;
        t.linear.x = v;
        cmd_vel_pub_->publish(t);
    }

    // シリンダ状態[front,rear]を指令して hold 秒保持 → 次へ
    void climb_stow(int front, int rear, double hold, ClimbStep next, const char* tag)
    {
        stop_robot();
        if (!climb_cmd_sent_) {
            publish_cylinder(front, rear);
            climb_cmd_sent_ = true;
            RCLCPP_INFO(get_logger(), "%s (front=%d rear=%d, hold=%.1fs)", tag, front, rear, hold);
            return;
        }
        if (step_elapsed() > hold) enter_climb_step(next);
    }

    // ── ユーティリティ ──────────────────────────
    void publish_cylinder(int front, int rear)
    {
        std_msgs::msg::Int16MultiArray m;
        m.data = {static_cast<int16_t>(front), static_cast<int16_t>(rear)};
        cylinder_pub_->publish(m);
    }

    void emit_done(const std::string& phase)
    {
        stop_robot();
        std_msgs::msg::String m; m.data = phase;
        phase_done_pub_->publish(m);
        done_emitted_ = true;
        RCLCPP_INFO(get_logger(), "phase_done → %s", phase.c_str());
    }

    void stop_robot()
    {
        geometry_msgs::msg::Twist t;
        cmd_vel_pub_->publish(t);
    }

    bool wall_dist_fresh()  const { return stamp_fresh(wall_dist_stamp_); }
    bool wall_angle_fresh() const { return stamp_fresh(wall_angle_stamp_); }
    bool imu_fresh()        const { return stamp_fresh(imu_stamp_); }
    bool odom_fresh()       const { return stamp_fresh(odom_stamp_); }

    bool stamp_fresh(const rclcpp::Time& t) const
    {
        if (t.nanoseconds() == 0) return false;
        return (now() - t).seconds() < sensor_timeout_;
    }

    static double wrap_pi(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    // ── パラメータ ─────────────────────────────
    int    confirm_frames_;
    double min_state_dwell_, sensor_timeout_;
    double target_distance_, approach_v_, approach_kp_, approach_timeout_;
    double align_sign_, align_tol_, align_kp_, align_omega_max_, align_timeout_;
    double raise_hold_, fwd_front_dist_, fwd_front_v_, fwd_front_timeout_, stow_front_hold_;
    double fwd_rear_dist_, fwd_rear_v_, fwd_rear_timeout_, stow_rear_hold_;
    double shock_thresh_, shock_cooldown_;
    bool   require_shock_;
    double rate_hz_;

    // ── 実行状態 ───────────────────────────────
    std::string  phase_ = "IDLE";
    rclcpp::Time phase_entered_{0,0,RCL_ROS_TIME};
    bool done_emitted_ = false;

    double wall_dist_  = 0.0; rclcpp::Time wall_dist_stamp_{0,0,RCL_ROS_TIME};
    double wall_angle_ = 0.0; rclcpp::Time wall_angle_stamp_{0,0,RCL_ROS_TIME};
    double imu_yaw_ = 0.0, imu_pitch_ = 0.0; rclcpp::Time imu_stamp_{0,0,RCL_ROS_TIME};
    double odom_x_ = 0.0, odom_y_ = 0.0; rclcpp::Time odom_stamp_{0,0,RCL_ROS_TIME};

    int reach_count_ = 0, align_count_ = 0;
    bool   align_target_set_ = false;
    double align_target_yaw_ = 0.0;

    // CLIMBサブシーケンス
    ClimbStep    climb_step_ = ClimbStep::RAISE;
    rclcpp::Time climb_step_entered_{0,0,RCL_ROS_TIME};
    bool   climb_cmd_sent_ = false;
    double fwd_start_x_ = 0.0, fwd_start_y_ = 0.0;

    // 衝撃検知
    bool   shock_prev_over_ = false;
    bool   shock_seen_this_step_ = false;
    int    shock_total_ = 0;
    rclcpp::Time last_shock_time_{0,0,RCL_ROS_TIME};

    // ── ROS ────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Int16MultiArray>::SharedPtr cylinder_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr phase_done_pub_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr  state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr dist_sub_, angle_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr  imu_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NatsuClimbSeqNode>());
    rclcpp::shutdown();
    return 0;
}
