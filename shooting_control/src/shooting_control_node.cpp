/**
 * shooting_control_node.cpp
 *
 * 役割:
 *   wall_detection と cage_detection の出力を受け取り、
 *   射出機構（フライホイール + 仰角サーボ）へのコマンドを計算して publish する。
 *   「いつ撃つか」の判断はしない。FSMノード（後述）が /shooter/fire を
 *   true にしたタイミングで初めて fire フラグを立てる。
 *
 * 弾道モデル:
 *   スポンジ棒（小うなぎ SGB-300）を剛体射体として扱い、
 *   空気抵抗なし放物線モデルで初速 v0 と仰角 θ を求める。
 *
 *   水平距離 d = pos3d.z (カメラ座標系の前方距離) [m]
 *   高さ差   h = cage_height_m - shooter_height_m    [m]
 *
 *   v0 と θ の関係（発射点→カゴ中心の放物線）:
 *     d = v0² sin(2θ) / g   (h=0 近似)
 *     実際は h≠0 なので数値的に解く（後述）
 *
 *   フライホイール回転数 RPM は v0 と線形マップで変換:
 *     rpm = rpm_per_ms * v0 * 1000   (v0 [m/s])
 *   係数は実機キャリブレーションで決定する。
 *
 * subscribeトピック:
 *   /wall_detection/angle    [std_msgs/Float64]  壁偏角 [rad]
 *   /wall_detection/distance [std_msgs/Float64]  壁距離 [m]
 *   /cage_detection/target   [cage_detection/Cage] 最優先カゴ
 *   /shooter/fire_request    [std_msgs/Bool]     FSMからの発射許可
 *
 * publishトピック:
 *   /shooter/command         [shooting_control/ShooterCommand]
 *   /shooter/ready           [std_msgs/Bool]  射撃準備完了フラグ
 */

#include <cmath>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "cage_detection/msg/cage.hpp"
#include "shooting_control/msg/shooter_command.hpp"

// ──────────────────────────────────────────────
//  弾道ソルバー（内部）
// ──────────────────────────────────────────────
namespace ballistics {

constexpr double G = 9.80665; // [m/s²](桁数はテキトー、要適宜調整)

struct Solution {
    double elevation_rad; // 仰角 [rad]
    double v0;            // 初速 [m/s]
};

/**
 * 水平距離 d [m]、高さ差 h [m]（カゴ - 発射点）、
 * 仰角候補 theta_rad を与えて実際の着弾距離を返す。
 */
double range_at_angle(double v0, double theta, double h)
{
    // 着弾時間 t を求める: h = v0 sin(θ)t - 0.5 g t²
    // → 0.5g t² - v0 sin(θ) t + h = 0
    const double vy = v0 * std::sin(theta);
    const double disc = vy * vy - 2.0 * G * h;
    if (disc < 0) return -1.0; // 届かない
    const double t = (vy + std::sqrt(disc)) / G;
    return v0 * std::cos(theta) * t;
}

/**
 * 水平距離 d、高さ差 h に対して
 * 固定仰角 theta_fixed で必要な v0 を2分探索で求める。
 * 久々にアルゴリズムを使えて嬉しいです。
 *
 * 仰角固定（サーボがキャリブレーション済みで1点の場合）または
 * 初速固定（フライホイール回転数上限がある場合）で使い分ける。
 *
 * ここでは「仰角を距離に応じて変化させる」戦略を採用:
 *   - v0 は max_v0 固定
 *   - theta を二分探索で求める
 */
std::optional<Solution> solve(
    double d, double h,
    double max_v0,
    double theta_min_rad, double theta_max_rad,
    int iterations = 40)
{
    // theta_min で届くか確認
    if (range_at_angle(max_v0, theta_min_rad, h) < d * 0.5) {
        return std::nullopt; // 最大速度でも届かない
    }

    // 二分探索: range(theta) = d を満たす theta を探す
    // range は theta が増えると先に増え後に減る（45°で最大）
    // → 低角解（theta < 45°）を使う（反応が早い）
    double lo = theta_min_rad;
    double hi = std::min(theta_max_rad, M_PI / 4.0);

    for (int i = 0; i < iterations; ++i) {
        double mid = (lo + hi) / 2.0;
        double r   = range_at_angle(max_v0, mid, h);
        if (r < 0) { lo = mid; continue; }
        if (r < d) lo = mid;
        else       hi = mid;
    }

    const double theta = (lo + hi) / 2.0;
    return Solution{theta, max_v0};
}

} // namespace ballistics

// ──────────────────────────────────────────────
//  ノード本体
// ──────────────────────────────────────────────
class ShootingControlNode : public rclcpp::Node
{
public:
    ShootingControlNode() : Node("shooting_control_node")
    {
        // ── パラメータ ───────────────────────────────
        // 機体形状
        declare_parameter<double>("shooter_height_m",    0.40);  // 発射口の地上高 [m]
        declare_parameter<double>("cage_rim_height_m",   0.37);  // カゴ上端の高さ [m]
                                                                  // 石倉(0.101m) + コンテナ高さ
        // 弾道
        declare_parameter<double>("max_v0",              6.0);   // 最大初速 [m/s]
        declare_parameter<double>("elevation_min_deg",   5.0);  // 仰角下限 
        declare_parameter<double>("elevation_max_deg",   45.0);  // 仰角上限(これ以上は意味がない)

        // フライホイール変換係数 [RPM / (m/s)]
        // 実機キャリブで決める。初期値は仮置き
        declare_parameter<double>("rpm_per_v0",          800.0);
        declare_parameter<double>("flywheel_idle_rpm",   0.0);   // 待機回転数

        // 正対判定閾値（FSMが使うが参照用にここでも持つ）
        declare_parameter<double>("align_angle_thresh_rad", 0.05); // ≈3°
        declare_parameter<double>("align_dist_thresh_m",    0.10); // ±10cm

        // 射出後の待機時間 [s]（フライホイール停止ディレイ）
        declare_parameter<double>("post_fire_wait_s",    1.0);

        load_params();

        // ── Publisher ───────────────────────────────
        cmd_pub_   = create_publisher<shooting_control::msg::ShooterCommand>(
            "/shooter/command", 10);
        ready_pub_ = create_publisher<std_msgs::msg::Bool>(
            "/shooter/ready", 10);

        // ── Subscriber ──────────────────────────────
        angle_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){ wall_angle_ = m->data; });

        dist_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/distance", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){ wall_dist_ = m->data; });

        cage_sub_ = create_subscription<cage_detection::msg::Cage>(
            "/cage_detection/target", 10,
            std::bind(&ShootingControlNode::cage_callback, this, std::placeholders::_1));

        fire_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/shooter/fire_request", 10,
            std::bind(&ShootingControlNode::fire_callback, this, std::placeholders::_1));

        // ── タイマー（制御ループ 20Hz） ──────────────
        timer_ = create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&ShootingControlNode::control_loop, this));

        RCLCPP_INFO(get_logger(), "shooting_control_node started.");
    }

private:
    void load_params()
    {
        shooter_h_    = get_parameter("shooter_height_m").as_double();
        cage_rim_h_   = get_parameter("cage_rim_height_m").as_double();
        max_v0_       = get_parameter("max_v0").as_double();
        elev_min_     = get_parameter("elevation_min_deg").as_double() * M_PI / 180.0;
        elev_max_     = get_parameter("elevation_max_deg").as_double() * M_PI / 180.0;
        rpm_per_v0_   = get_parameter("rpm_per_v0").as_double();
        idle_rpm_     = get_parameter("flywheel_idle_rpm").as_double();
        align_angle_  = get_parameter("align_angle_thresh_rad").as_double();
        align_dist_   = get_parameter("align_dist_thresh_m").as_double();
        post_fire_w_  = get_parameter("post_fire_wait_s").as_double();
    }

    // ── カゴ情報受信 ──────────────────────────────
    void cage_callback(const cage_detection::msg::Cage::SharedPtr msg)
    {
        latest_cage_ = *msg;
        cage_valid_  = (msg->distance > 0.3 && msg->distance < 6.0);
    }

    // ── 発射許可受信 ──────────────────────────────
    void fire_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (msg->data && !fire_requested_) {
            RCLCPP_INFO(get_logger(), "Fire request received.");
        }
        fire_requested_ = msg->data;
    }

    // ── メイン制御ループ（20Hz） ──────────────────
    void control_loop()
    {
        if (!cage_valid_) {
            publish_idle();
            return;
        }

        // ── 弾道計算 ─────────────────────────────
        // 高さ差: カゴ上端 - 発射口
        // カゴはエリア2（ブルーシート上）の地面に置かれている
        // 石倉（101mm）+ カゴ高さ の差を引く
        // 発射口は石倉の上にある（shooter_h_ はロボット床面からの高さ）
        const double cage_height_above_floor = cage_rim_h_;
        const double shooter_height_above_floor =
            0.101 + shooter_h_; // 石倉高さ + 発射口高さ
        const double h = cage_height_above_floor - shooter_height_above_floor;

        const double d = latest_cage_.distance; // カメラz距離 [m]

        auto sol = ballistics::solve(d, h, max_v0_, elev_min_, elev_max_);

        if (!sol) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                "Ballistics solver failed: d=%.2f h=%.2f", d, h);
            publish_idle();
            return;
        }

        // ── 正対チェック ─────────────────────────
        // 壁偏角とwall_distが有効なら正対を確認
        const bool aligned = (std::abs(wall_angle_) < align_angle_);
        const bool ready   = aligned && cage_valid_ && sol.has_value();

        // ── コマンド生成 ──────────────────────────
        shooting_control::msg::ShooterCommand cmd;
        cmd.header.stamp    = now();
        cmd.elevation_rad   = sol->elevation_rad;
        cmd.flywheel_rpm    = sol->v0 * rpm_per_v0_;
        cmd.fire            = fire_requested_ && ready;
        cmd.target_cage_color   = latest_cage_.color;
        cmd.target_distance_m   = d;

        // 発射後フラグリセット（1回限り）
        if (cmd.fire) {
            fire_requested_ = false;
            RCLCPP_INFO(get_logger(),
                "FIRE: elev=%.1f° rpm=%.0f dist=%.2fm",
                sol->elevation_rad * 180.0 / M_PI,
                cmd.flywheel_rpm, d);
        }

        cmd_pub_->publish(cmd);

        // readyフラグ
        std_msgs::msg::Bool ready_msg;
        ready_msg.data = ready;
        ready_pub_->publish(ready_msg);

        RCLCPP_DEBUG(get_logger(),
            "elev=%.1f° rpm=%.0f dist=%.2f aligned=%d ready=%d",
            sol->elevation_rad * 180.0 / M_PI,
            cmd.flywheel_rpm, d,
            static_cast<int>(aligned),
            static_cast<int>(ready));
    }

    // ── アイドル出力 ──────────────────────────────
    void publish_idle()
    {
        shooting_control::msg::ShooterCommand cmd;
        cmd.header.stamp  = now();
        cmd.elevation_rad = elev_min_; // サーボを最小角で待機
        cmd.flywheel_rpm  = idle_rpm_;
        cmd.fire          = false;
        cmd_pub_->publish(cmd);

        std_msgs::msg::Bool ready_msg;
        ready_msg.data = false;
        ready_pub_->publish(ready_msg);
    }

    // ── メンバ変数 ──────────────────────────────
    // パラメータ
    double shooter_h_, cage_rim_h_;
    double max_v0_, elev_min_, elev_max_;
    double rpm_per_v0_, idle_rpm_;
    double align_angle_, align_dist_;
    double post_fire_w_;

    // 状態
    double  wall_angle_    = 0.0;
    double  wall_dist_     = 0.0;
    bool    cage_valid_    = false;
    bool    fire_requested_= false;
    cage_detection::msg::Cage latest_cage_;

    // ROS
    rclcpp::Publisher<shooting_control::msg::ShooterCommand>::SharedPtr cmd_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr                   ready_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr             dist_sub_;
    rclcpp::Subscription<cage_detection::msg::Cage>::SharedPtr          cage_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr                fire_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

// ──────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ShootingControlNode>());
    rclcpp::shutdown();
    return 0;
}
