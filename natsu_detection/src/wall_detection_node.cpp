//////////////////////////////////////
//　ノード一覧：
//  <サブスクライブ>
//
//
//  <パブリッシュ>
//  /wall_detection/angle    : 壁の角度（ロボット正面からの偏角、単位はラジアン）
//  /wall_detection/distance : 壁までの距離（単位はメートル）
//
///////////////////////////////////////


#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

// ──────────────────────────────────────────────
//  内部データ型
// ──────────────────────────────────────────────
struct Point2D { double x, y; };

struct LineCoeff {
    double a, b, c;   // ax + by + c = 0, normalized (a^2+b^2 == 1)
    int    inliers;
};

// ──────────────────────────────────────────────
//  ノード本体
// ──────────────────────────────────────────────
class WallDetectionNode : public rclcpp::Node
{
public:
    WallDetectionNode() : Node("wall_detection_node"), rng_(std::random_device{}())
    {
        // ── パラメータ宣言 ──────────────────────────
        declare_parameter<double>("fov_half_deg",60.0);
        declare_parameter<double>("aiming_fov_half_deg",20.0); 
        declare_parameter<bool>  ("aiming_mode",false); 
        declare_parameter<double>("max_angle_step_deg",5.0);
        declare_parameter<double>("distance_gate_min",0.10); 
        declare_parameter<double>("distance_gate_max",4.00);
        declare_parameter<int>   ("ransac_iterations",50);
        declare_parameter<double>("ransac_threshold",0.03);
        declare_parameter<int>   ("ransac_min_inliers",8);
        declare_parameter<double>("min_inlier_ratio",0.5);
        declare_parameter<double>("line_lpf_alpha",0.3);
        declare_parameter<double>("angle_lpf_alpha",0.2);
        declare_parameter<double>("dist_lpf_alpha",0.2);
        declare_parameter<double>("warm_start_threshold_deg",3.0); 
        declare_parameter<double>("angle_offset_deg",0.0);   

        load_params();

        //Publisher
        angle_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/wall_detection/angle",10);
        distance_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/wall_detection/distance",10);

        //Subscriber
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", rclcpp::SensorDataQoS(),
            std::bind(&WallDetectionNode::scan_callback, this, std::placeholders::_1));

        aiming_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/wall_detection/aiming_mode",10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                aiming_mode_ = msg->data;
                RCLCPP_INFO(get_logger(), "aiming_mode -> %s", aiming_mode_ ? "ON" : "OFF");
            });

        RCLCPP_INFO(get_logger(), "wall_detection_node (夏ロボ改修版) started.");
    }

private:
    //パラメータ読み込み
    void load_params()
    {
        fov_half_rad_        = get_parameter("fov_half_deg").as_double()        * M_PI / 180.0;
        aiming_fov_half_rad_ = get_parameter("aiming_fov_half_deg").as_double() * M_PI / 180.0;
        aiming_mode_         = get_parameter("aiming_mode").as_bool();
        max_angle_step_rad_  = get_parameter("max_angle_step_deg").as_double()  * M_PI / 180.0;
        distance_gate_min_   = get_parameter("distance_gate_min").as_double();
        distance_gate_max_   = get_parameter("distance_gate_max").as_double();
        ransac_iterations_   = get_parameter("ransac_iterations").as_int();
        ransac_threshold_    = get_parameter("ransac_threshold").as_double();
        ransac_min_inliers_  = get_parameter("ransac_min_inliers").as_int();
        min_inlier_ratio_    = get_parameter("min_inlier_ratio").as_double();
        line_lpf_alpha_      = get_parameter("line_lpf_alpha").as_double();
        angle_lpf_alpha_     = get_parameter("angle_lpf_alpha").as_double();
        dist_lpf_alpha_      = get_parameter("dist_lpf_alpha").as_double();
        warm_start_threshold_= get_parameter("warm_start_threshold_deg").as_double() * M_PI / 180.0;
        angle_offset_rad_    = get_parameter("angle_offset_deg").as_double()    * M_PI / 180.0;
    }

    // ── メインコールバック ──────────────────────
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 1. デカルト変換 + FOVフィルタ + 距離ゲート
        const double active_fov = aiming_mode_ ? aiming_fov_half_rad_ : fov_half_rad_;
        auto pts = filter_points(*scan, active_fov);

        if (static_cast<int>(pts.size()) < ransac_min_inliers_) {
            RCLCPP_DEBUG(get_logger(), "Not enough points in FOV: %zu", pts.size());
            return;
        }

        // 2. RANSAC（ウォームスタートで可能ならスキップ） + TLS再フィット
        std::optional<LineCoeff> result;
        if (warm_start_valid(pts)) {
            result = refine_line_with_inliers(pts, smoothed_line_.value());
        } else {
            auto ransac_res = ransac_fit(pts);
            if (!ransac_res) return;
            const double ratio = static_cast<double>(ransac_res->inliers) / pts.size();
            if (ransac_res->inliers < ransac_min_inliers_ || ratio < min_inlier_ratio_) return;
            result = refine_line_with_inliers(pts, *ransac_res);
        }
        if (!result) return;

        // 3. LPF（修正版） + publish
        smooth_and_publish(*result);
    }

    // ── FOVフィルタ + 距離ゲート ────────────────
    std::vector<Point2D> filter_points(
        const sensor_msgs::msg::LaserScan& scan,
        double fov_half_rad) const
    {
        std::vector<Point2D> pts;
        pts.reserve(512);

        double prev_angle = std::numeric_limits<double>::quiet_NaN();
        const size_t n = scan.ranges.size();

        for (size_t i = 0; i < n; ++i) {
            const double r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

            // [Add 1] 距離ゲート
            if (r < distance_gate_min_ || r > distance_gate_max_) continue;

            const double angle = scan.angle_min + i * scan.angle_increment;

            // FOVフィルタ（前方 ±fov_half_rad）
            if (std::abs(angle) > fov_half_rad) continue;

            // 連続点間の角度ステップ異常除去
            if (!std::isnan(prev_angle) &&
                std::abs(angle - prev_angle) > max_angle_step_rad_) {
                prev_angle = angle;
                continue;
            }
            prev_angle = angle;

            pts.push_back({r * std::cos(angle), r * std::sin(angle)});
        }
        return pts;
    }

    // ── RANSAC ────────────────────────────────
    std::optional<LineCoeff> ransac_fit(const std::vector<Point2D>& pts)
    {
        if (pts.size() < 2) return std::nullopt;

        LineCoeff best{0, 0, 0, 0};
        std::uniform_int_distribution<size_t> dist(0, pts.size() - 1);

        for (int iter = 0; iter < ransac_iterations_; ++iter) {
            size_t i = dist(rng_), j;
            do { j = dist(rng_); } while (j == i);

            const auto& p1 = pts[i];
            const auto& p2 = pts[j];
            double a = p2.y - p1.y;
            double b = p1.x - p2.x;
            double c = -(a * p1.x + b * p1.y);
            const double norm = std::sqrt(a*a + b*b);
            if (norm < 1e-9) continue;
            a /= norm; b /= norm; c /= norm;

            int inliers = 0;
            for (const auto& p : pts) {
                if (std::abs(a*p.x + b*p.y + c) < ransac_threshold_) ++inliers;
            }
            if (inliers > best.inliers) best = {a, b, c, inliers};
        }
        if (best.inliers == 0) return std::nullopt;
        return best;
    }

    // ── TLS再フィット ──────────────────────────
    LineCoeff refine_line_with_inliers(
        const std::vector<Point2D>& pts,
        const LineCoeff& seed) const
    {
        // インライア点を抽出
        std::vector<Point2D> inliers;
        inliers.reserve(pts.size());
        for (const auto& p : pts) {
            if (std::abs(seed.a*p.x + seed.b*p.y + seed.c) < ransac_threshold_) {
                inliers.push_back(p);
            }
        }
        if (inliers.size() < 2) return seed;

        // 重心
        double mx = 0, my = 0;
        for (const auto& p : inliers) { mx += p.x; my += p.y; }
        mx /= inliers.size(); my /= inliers.size();

        // 共分散行列
        double sxx = 0, sxy = 0, syy = 0;
        for (const auto& p : inliers) {
            double dx = p.x - mx, dy = p.y - my;
            sxx += dx*dx; sxy += dx*dy; syy += dy*dy;
        }

        // 最小固有値の固有ベクトル = 法線方向
        double diff = (sxx - syy) / 2.0;
        double disc = std::sqrt(diff*diff + sxy*sxy);
        double lambda_min = (sxx + syy) / 2.0 - disc;

        double na = sxy;
        double nb = lambda_min - sxx;
        double nn = std::sqrt(na*na + nb*nb);
        if (nn < 1e-9) return seed;
        na /= nn; nb /= nn;

        // c = -n・(重心)
        double nc = -(na*mx + nb*my);

        // 法線を x > 0 方向に統一
        if (na < 0) { na = -na; nb = -nb; nc = -nc; }

        return {na, nb, nc, static_cast<int>(inliers.size())};
    }

    // ── ウォームスタート判定 [Add 2] ─────────────
    bool warm_start_valid(const std::vector<Point2D>& pts) const
    {
        if (!smoothed_line_.has_value()) return false;
        const auto& L = smoothed_line_.value();

        // 前フレーム直線に対するインライア率チェック
        int inliers = 0;
        for (const auto& p : pts) {
            if (std::abs(L.a*p.x + L.b*p.y + L.c) < ransac_threshold_ * 1.5) ++inliers;
        }
        const double ratio = static_cast<double>(inliers) / pts.size();
        return ratio >= min_inlier_ratio_;
    }

    // ── LPF平滑化 + publish [Fix 1] [Fix 2] ─────
    void smooth_and_publish(const LineCoeff& raw)
    {
        // ── 角度LPF（a,bのみ平滑化→再正規化）
        double sa, sb;
        if (!smoothed_line_.has_value()) {
            sa = raw.a; sb = raw.b;
        } else {
            sa = (1.0 - line_lpf_alpha_) * smoothed_line_->a + line_lpf_alpha_ * raw.a;
            sb = (1.0 - line_lpf_alpha_) * smoothed_line_->b + line_lpf_alpha_ * raw.b;
        }
        const double sn = std::sqrt(sa*sa + sb*sb);
        if (sn < 1e-9) return;
        sa /= sn; sb /= sn;

        // a,bから再計算したcは使わず、raw.cを独立LPFで平滑化 [Fix 1]
        const double raw_dist = std::abs(raw.c);
        if (!std::isfinite(filtered_distance_)) {
            filtered_distance_ = raw_dist;
        } else {
            filtered_distance_ = (1.0 - dist_lpf_alpha_) * filtered_distance_
                                + dist_lpf_alpha_ * raw_dist;
        }

        // smoothed_line_ を更新（cは平滑化距離で再設定）
        // 符号はロボット原点が直線の正側になるよう合わせる
        const double sc = -(sa * 0.0 + sb * 0.0) - filtered_distance_;
        // ※ 正確には重心からcを取る。ここでは距離だけ保持しLPFを通した値を使う
        smoothed_line_ = LineCoeff{sa, sb, -filtered_distance_, raw.inliers};

        // ── 壁角度の計算
        // 法線方向(na,nb)とx軸の角度 → ロボットの正面方向からの偏角
        const double raw_wall_angle = std::atan2(sb, sa);

        // 角度独立LPF
        if (!std::isfinite(filtered_angle_)) {
            filtered_angle_ = raw_wall_angle;
        } else {
            // 角度折り返し対応
            double diff = raw_wall_angle - filtered_angle_;
            while (diff >  M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
            filtered_angle_ += angle_lpf_alpha_ * diff;
        }

        // 取り付けオフセット補正 [Add 4]
        const double output_angle = filtered_angle_ + angle_offset_rad_;

        // ── publish
        auto angle_msg = std_msgs::msg::Float64();
        angle_msg.data = output_angle;
        angle_pub_->publish(angle_msg);

        auto dist_msg = std_msgs::msg::Float64();   // [Fix 2]
        dist_msg.data = filtered_distance_;
        distance_pub_->publish(dist_msg);

        RCLCPP_DEBUG(get_logger(),
            "wall angle=%.3f rad (%.1f deg), dist=%.3f m, inliers=%d",
            output_angle, output_angle * 180.0 / M_PI,
            filtered_distance_, raw.inliers);
    }

    // ── メンバ変数 ──────────────────────────────
    // パラメータ
    double fov_half_rad_, aiming_fov_half_rad_;
    bool   aiming_mode_;
    double max_angle_step_rad_;
    double distance_gate_min_, distance_gate_max_;
    int    ransac_iterations_, ransac_min_inliers_;
    double ransac_threshold_, min_inlier_ratio_;
    double line_lpf_alpha_, angle_lpf_alpha_, dist_lpf_alpha_;
    double warm_start_threshold_;
    double angle_offset_rad_;

    // 状態
    std::optional<LineCoeff> smoothed_line_;
    double filtered_angle_    = std::numeric_limits<double>::quiet_NaN();
    double filtered_distance_ = std::numeric_limits<double>::quiet_NaN();

    // ROS
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   distance_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         aiming_sub_;

    std::mt19937 rng_;
};

// ──────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
