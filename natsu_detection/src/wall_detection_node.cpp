//////////////////////////////////////
//　ノード一覧：
//  <サブスクライブ>
//  /scan                     : LiDARスキャンデータ
//  /wall_detection/aiming_mode : エイミングモードのON/OFF（bool）
//
//
//  <パブリッシュ>
//  /wall_detection/angle    : 壁の角度（ロボット正面からの偏角、単位はラジアン）
//  /wall_detection/distance : 壁までの距離（単位はメートル）
//  /wall_detection/filtered_points : フィルタ後の点群(GUI/RViz用)
//  /wall_detection/ransac_params   : 採用した直線 [a,b,c,inliers,total]
//
//  石倉は凹形なので、FOV内には「左の側面・奥の面・右の側面」が同時に入る。
//  単一直線のRANSACだと点数の多い側面に引っかかって明後日の方向に平行判定が
//  出るため、逐次RANSACで複数の面を抜き出し、その中から「正面を向いている面」
//  (法線がロボット前方に近い面)だけを壁として採用する。
///////////////////////////////////////


#include <cmath>
#include <vector>
#include <random>
#include <algorithm>
#include <optional>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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

        // ── [ROI追加] 矩形マスク（ロボット基準・LiDAR座標系）──────
        // 石倉の側面以外(フィールドの木枠など)を空間で除外する。
        // LiDAR原点から見て x=前方[m], y=左方向[m]。この箱の中の点だけ残す。
        // 石倉正対シーケンス前提なので、石倉が来る前方範囲を決め打ちできる。
        // use_roi=false で従来通り(マスク無効)。実測して4辺を詰めること。
        declare_parameter<bool>  ("use_roi", false);
        declare_parameter<double>("roi_x_min", 0.10);   // 前方 近い側 [m]
        declare_parameter<double>("roi_x_max", 1.50);   // 前方 遠い側 [m]
        declare_parameter<double>("roi_y_min",-0.80);   // 右端 [m]
        declare_parameter<double>("roi_y_max", 0.80);   // 左端 [m]

        // 凹形対応: 逐次RANSACで抜き出す面の最大数
        declare_parameter<int>   ("max_wall_candidates",3);
        // 法線の偏角がこれを超える面は「正面の壁ではない」として捨てる [deg]
        // 石倉の側面(≒±90°)を弾くための本命パラメータ
        declare_parameter<double>("wall_angle_max_deg",40.0);
        // ウォームスタートを連続で使う上限フレーム数(誤ロックの固定化を防ぐ)
        declare_parameter<int>   ("warm_start_max_frames",20);

        load_params();

        //Publisher
        angle_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/wall_detection/angle",10);
        distance_pub_ = create_publisher<std_msgs::msg::Float64>(
            "/wall_detection/distance",10);
        // GUI(点群パネル / 自己位置マップ重畳) と RViz 用
        filtered_points_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
            "/wall_detection/filtered_points",10);
        ransac_params_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(
            "/wall_detection/ransac_params",10);

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

        RCLCPP_INFO(get_logger(), "wall_detection_node started.");
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
        // [ROI追加]
        use_roi_             = get_parameter("use_roi").as_bool();
        roi_x_min_           = get_parameter("roi_x_min").as_double();
        roi_x_max_           = get_parameter("roi_x_max").as_double();
        roi_y_min_           = get_parameter("roi_y_min").as_double();
        roi_y_max_           = get_parameter("roi_y_max").as_double();
        ransac_iterations_   = get_parameter("ransac_iterations").as_int();
        ransac_threshold_    = get_parameter("ransac_threshold").as_double();
        ransac_min_inliers_  = get_parameter("ransac_min_inliers").as_int();
        min_inlier_ratio_    = get_parameter("min_inlier_ratio").as_double();
        line_lpf_alpha_      = get_parameter("line_lpf_alpha").as_double();
        angle_lpf_alpha_     = get_parameter("angle_lpf_alpha").as_double();
        dist_lpf_alpha_      = get_parameter("dist_lpf_alpha").as_double();
        warm_start_threshold_= get_parameter("warm_start_threshold_deg").as_double() * M_PI / 180.0;
        angle_offset_rad_    = get_parameter("angle_offset_deg").as_double()    * M_PI / 180.0;
        max_wall_candidates_ = get_parameter("max_wall_candidates").as_int();
        wall_angle_max_rad_  = get_parameter("wall_angle_max_deg").as_double()  * M_PI / 180.0;
        warm_start_max_frames_ = get_parameter("warm_start_max_frames").as_int();
    }

    // ── メインコールバック ──────────────────────
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan)
    {
        // 1. デカルト変換 + FOVフィルタ + 距離ゲート
        const double active_fov = aiming_mode_ ? aiming_fov_half_rad_ : fov_half_rad_;
        auto pts = filter_points(*scan, active_fov);

        // フィルタ後の点群を常時 publish（壁が見つからなくても点は出す）
        publish_filtered_points(pts, scan->header);

        if (static_cast<int>(pts.size()) < ransac_min_inliers_) {
            RCLCPP_DEBUG(get_logger(), "Not enough points in FOV: %zu", pts.size());
            return;
        }

        // 2. 壁面の決定
        std::optional<LineCoeff> result = try_warm_start(pts);
        if (!result) {
            result = select_front_wall(pts);
            warm_start_frames_ = 0;
        }
        if (!result) {
            // 正面を向いた面が見つからない。前回値を出し続けると嘘をつくので黙る。
            RCLCPP_DEBUG(get_logger(), "正面の壁が見つからない (pts=%zu)", pts.size());
            return;
        }

        // 採用した直線パラメータを publish ([a, b, c, inliers, total])
        {
            std_msgs::msg::Float64MultiArray rp;
            rp.data = {result->a, result->b, result->c,
                       static_cast<double>(result->inliers),
                       static_cast<double>(pts.size())};
            ransac_params_pub_->publish(rp);
        }

        // 3. LPF + publish
        smooth_and_publish(*result);
    }

    // ── ウォームスタート ────────────────────────
    // 前フレームの面をそのまま追従する。誤った面にロックしたまま抜けられなく
    // ならないよう、(1) 角度が warm_start_threshold_ 以上飛んだら破棄、
    // (2) 連続 warm_start_max_frames_ 回で強制的に全探索へ戻す。
    std::optional<LineCoeff> try_warm_start(const std::vector<Point2D>& pts)
    {
        if (!smoothed_line_.has_value()) return std::nullopt;
        if (warm_start_frames_ >= warm_start_max_frames_) return std::nullopt;

        const auto& L = smoothed_line_.value();
        int inliers = 0;
        for (const auto& p : pts) {
            if (std::abs(L.a*p.x + L.b*p.y + L.c) < ransac_threshold_ * 1.5) ++inliers;
        }
        if (inliers < ransac_min_inliers_) return std::nullopt;

        const LineCoeff refined = refine_line_with_inliers(pts, L);
        const double ang = std::atan2(refined.b, refined.a);

        // 正面から外れた面は追従対象にしない
        if (std::abs(ang) > wall_angle_max_rad_) return std::nullopt;
        // 前フレームから角度が飛びすぎていたら信用しない
        if (std::isfinite(filtered_angle_) &&
            std::abs(wrap_pi(ang - filtered_angle_)) > warm_start_threshold_) {
            return std::nullopt;
        }

        ++warm_start_frames_;
        return refined;
    }

    // ── 正面の壁を選ぶ (凹形対応したか？) ──────────
    // 逐次RANSACで面を抜き出し、法線がロボット前方を向いている面のうち
    // もっとも支持点の多いものを壁として返す。
    std::optional<LineCoeff> select_front_wall(const std::vector<Point2D>& pts)
    {
        std::vector<Point2D> remaining = pts;
        std::optional<LineCoeff> best;

        for (int k = 0; k < max_wall_candidates_; ++k) {
            if (static_cast<int>(remaining.size()) < ransac_min_inliers_) break;

            auto seed = ransac_fit(remaining);
            if (!seed || seed->inliers < ransac_min_inliers_) break;

            const LineCoeff cand = refine_line_with_inliers(remaining, *seed);
            if (cand.inliers < ransac_min_inliers_) break;

            // この面の法線がロボット前方を向いているか
            const double ang = std::atan2(cand.b, cand.a);
            const bool facing = std::abs(ang) <= wall_angle_max_rad_;
            const double ratio = static_cast<double>(cand.inliers) / pts.size();

            if (facing && ratio >= min_inlier_ratio_ &&
                (!best || cand.inliers > best->inliers)) {
                best = cand;
            }

            RCLCPP_DEBUG(get_logger(),
                "候補%d: angle=%.1fdeg dist=%.2fm inliers=%d(%.0f%%) %s",
                k, ang * 180.0 / M_PI, std::abs(cand.c), cand.inliers, ratio * 100.0,
                facing ? "採用可" : "側面とみなし除外");

            // 採用した面の点を除いて次の面を探す
            std::vector<Point2D> rest;
            rest.reserve(remaining.size());
            for (const auto& p : remaining) {
                if (std::abs(cand.a*p.x + cand.b*p.y + cand.c) >= ransac_threshold_)
                    rest.push_back(p);
            }
            if (rest.size() == remaining.size()) break;  // 進捗なし
            remaining.swap(rest);
        }
        return best;
    }

    // フィルタ後点群を PointCloud2(x,y,z=0, LiDARフレーム) として publish
    void publish_filtered_points(const std::vector<Point2D>& pts,
                                 const std_msgs::msg::Header& header)
    {
        sensor_msgs::msg::PointCloud2 cloud;
        cloud.header = header;          // frame_id=ldlidar_link, stamp はスキャン由来
        cloud.height = 1;
        cloud.width  = static_cast<uint32_t>(pts.size());
        cloud.is_dense = true;
        sensor_msgs::PointCloud2Modifier mod(cloud);
        mod.setPointCloud2FieldsByString(1, "xyz");
        mod.resize(pts.size());
        sensor_msgs::PointCloud2Iterator<float> ix(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> iy(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> iz(cloud, "z");
        for (const auto& p : pts) {
            *ix = static_cast<float>(p.x);
            *iy = static_cast<float>(p.y);
            *iz = 0.0f;
            ++ix; ++iy; ++iz;
        }
        filtered_points_pub_->publish(cloud);
    }

    // ── FOVフィルタ + 距離ゲート ────────────────
    std::vector<Point2D> filter_points(
        const sensor_msgs::msg::LaserScan& scan,
        double fov_half_rad) const
    {
        std::vector<Point2D> pts;
        pts.reserve(512);

        const size_t n = scan.ranges.size();

        for (size_t i = 0; i < n; ++i) {
            const double r = scan.ranges[i];
            if (!std::isfinite(r) || r < scan.range_min || r > scan.range_max) continue;

            // [Add 1] 距離ゲート
            if (r < distance_gate_min_ || r > distance_gate_max_) continue;

            const double angle = scan.angle_min + i * scan.angle_increment;

            // FOVフィルタ（前方 ±fov_half_rad）
            if (std::abs(angle) > fov_half_rad) continue;

            const double px = r * std::cos(angle);   // 前方[m]
            const double py = r * std::sin(angle);   // 左方向[m]

            // [ROI追加] 矩形マスク: 石倉周辺の箱の中だけ残す。
            // フィールドの木枠など、箱の外にある壁をここで捨てる。
            if (use_roi_) {
                if (px < roi_x_min_ || px > roi_x_max_ ||
                    py < roi_y_min_ || py > roi_y_max_) {
                    continue;
                }
            }

            pts.push_back({px, py});
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
        // 法線はx>0側に正規化済みなので、原点からの符号付き距離は -filtered_distance_
        smoothed_line_ = LineCoeff{sa, sb, -filtered_distance_, raw.inliers};

        // ── 壁角度の計算
        // 法線方向(na,nb)とx軸の角度 → ロボットの正面方向からの偏角
        const double raw_wall_angle = std::atan2(sb, sa);

        // 角度独立LPF
        if (!std::isfinite(filtered_angle_)) {
            filtered_angle_ = raw_wall_angle;
        } else {
            filtered_angle_ += angle_lpf_alpha_ * wrap_pi(raw_wall_angle - filtered_angle_);
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

    static double wrap_pi(double a)
    {
        while (a >  M_PI) a -= 2 * M_PI;
        while (a < -M_PI) a += 2 * M_PI;
        return a;
    }

    // ── メンバ変数 ──────────────────────────────
    // パラメータ
    double fov_half_rad_, aiming_fov_half_rad_;
    bool   aiming_mode_;
    double max_angle_step_rad_;
    double distance_gate_min_, distance_gate_max_;
    // [ROI追加] 矩形マスク（ロボット基準）
    bool   use_roi_ = false;
    double roi_x_min_ = 0.10, roi_x_max_ = 1.50;
    double roi_y_min_ = -0.80, roi_y_max_ = 0.80;
    int    ransac_iterations_, ransac_min_inliers_;
    double ransac_threshold_, min_inlier_ratio_;
    double line_lpf_alpha_, angle_lpf_alpha_, dist_lpf_alpha_;
    double warm_start_threshold_;
    double angle_offset_rad_;
    int    max_wall_candidates_;
    double wall_angle_max_rad_;
    int    warm_start_max_frames_;

    // 状態
    std::optional<LineCoeff> smoothed_line_;
    double filtered_angle_    = std::numeric_limits<double>::quiet_NaN();
    double filtered_distance_ = std::numeric_limits<double>::quiet_NaN();
    int    warm_start_frames_ = 0;

    // ROS
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   angle_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr   distance_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr        filtered_points_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr     ransac_params_pub_;
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