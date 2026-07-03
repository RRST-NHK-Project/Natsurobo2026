// ════════════════════════════════════════════════════════════════════════
//  scan_matcher_localizer  —  RRST 夏ロボ 2026 自己位置推定
// ════════════════════════════════════════════════════════════════════════
//  LD19 360°LiDAR(/scan)を「既知フィールド壁線分」にマッチングし、
//  summer2026_odometry の /odom を初期値(動き予測)にして絶対自己位置を推定する。
//
//  地図は構築しない(SLAMではない)。フィールドが既知なので、壁を線分マップとして
//  与え、scan点を点-直線ICPで合わせる → 累積ドリフトが原理的に乗らない絶対位置。
//
//  アルゴリズムは LittleSLAM (MPL-2.0, Masahiro Tomono / fuRo) を移植・再構成:
//    Pose2D             … 2D姿勢と座標変換 (compose/relative/inverse)
//    PoseEstimatorICP   … 対応づけ↔最適化を収束まで反復
//    DataAssociator     … 各scan点に最近傍の既知壁線分を対応づけ
//    CostFunctionPD     … 点-直線の垂直距離をコストにする
//    PoseOptimizerGN    … ガウスニュートン法で姿勢を最適化
//    ScanMatcher2D      … 失敗時はオドメトリ予測にフォールバック
//
//  <Subscribe>  /scan (sensor_msgs/LaserScan),  odom (nav_msgs/Odometry)
//  <Publish>    /localization/pose (geometry_msgs/PoseWithCovarianceStamped, map)
//               TF: map -> odom
// ════════════════════════════════════════════════════════════════════════

#include <cmath>
#include <vector>
#include <limits>
#include <optional>

#include <Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"

namespace {

// ── 2D姿勢 (LittleSLAM Pose2D 相当) ───────────────────────────────────
struct Pose2D {
  double tx{0.0}, ty{0.0}, th{0.0};   // 並進x,y[m], 回転th[rad]
};

// 点を姿勢で地図座標系へ: X = R(th)·p + t  (globalPoint 相当)
inline void transformPoint(const Pose2D &ps, double px, double py, double &ox, double &oy) {
  const double c = std::cos(ps.th), s = std::sin(ps.th);
  ox = c * px - s * py + ps.tx;
  oy = s * px + c * py + ps.ty;
}

// 姿勢合成 npose = base ⊕ rel  (calGlobalPose 相当)
inline Pose2D compose(const Pose2D &base, const Pose2D &rel) {
  const double c = std::cos(base.th), s = std::sin(base.th);
  Pose2D o;
  o.tx = base.tx + c * rel.tx - s * rel.ty;
  o.ty = base.ty + s * rel.tx + c * rel.ty;
  o.th = std::atan2(std::sin(base.th + rel.th), std::cos(base.th + rel.th));
  return o;
}

// 相対姿勢 rel = base⁻¹ ⊕ npose  (calRelativePose 相当)
inline Pose2D relative(const Pose2D &npose, const Pose2D &base) {
  const double c = std::cos(base.th), s = std::sin(base.th);
  const double dx = npose.tx - base.tx, dy = npose.ty - base.ty;
  Pose2D o;
  o.tx = c * dx + s * dy;
  o.ty = -s * dx + c * dy;
  o.th = std::atan2(std::sin(npose.th - base.th), std::cos(npose.th - base.th));
  return o;
}

// 逆姿勢
inline Pose2D inverse(const Pose2D &p) {
  const double c = std::cos(p.th), s = std::sin(p.th);
  Pose2D o;
  o.tx = -(c * p.tx + s * p.ty);
  o.ty = -(-s * p.tx + c * p.ty);
  o.th = -p.th;
  return o;
}

// ── 既知壁の線分 (参照地図の要素) ─────────────────────────────────────
struct Segment {
  double ax, ay, bx, by;   // 端点
  double nx, ny;           // 単位法線
  double rho;              // 無限直線オフセット: n·A (residual = n·X - rho)
  double tx, ty;           // 単位接線
  double len;              // 長さ

  void precompute() {
    double dx = bx - ax, dy = by - ay;
    len = std::hypot(dx, dy);
    if (len < 1e-9) { tx = 1; ty = 0; nx = 0; ny = 1; }
    else { tx = dx / len; ty = dy / len; nx = -ty; ny = tx; }
    rho = nx * ax + ny * ay;
  }

  // 点(X)から線分への最近傍距離(対応づけ用, 端点でクランプ)
  double distToPoint(double x, double y) const {
    double u = (x - ax) * tx + (y - ay) * ty;
    if (u < 0) u = 0; else if (u > len) u = len;
    double cx = ax + u * tx, cy = ay + u * ty;
    return std::hypot(x - cx, y - cy);
  }
};

}  // namespace

// ════════════════════════════════════════════════════════════════════════

class ScanMatcherLocalizer : public rclcpp::Node {
public:
  ScanMatcherLocalizer() : Node("scan_matcher_localizer") {
    // ── パラメータ ──
    declare_parameter<std::vector<double>>("field_map.segments", std::vector<double>{});
    declare_parameter<std::vector<double>>("start_pose", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter<double>("lidar_x", 0.0);
    declare_parameter<double>("lidar_y", 0.0);
    declare_parameter<double>("lidar_yaw", 0.0);
    declare_parameter<std::string>("scan_topic", "/scan");
    declare_parameter<std::string>("odom_topic", "odom");
    declare_parameter<std::string>("imu_topic", "/imu");
    declare_parameter<std::string>("pose_topic", "/localization/pose");
    declare_parameter<bool>("publish_tf", true);
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<double>("range_min", 0.10);
    declare_parameter<double>("range_max", 12.0);
    declare_parameter<int>("point_skip", 2);
    declare_parameter<int>("icp_max_iter", 30);
    declare_parameter<double>("icp_dthre", 0.30);
    declare_parameter<double>("icp_dthre_end", 0.10);
    declare_parameter<int>("gn_max_iter", 5);
    declare_parameter<double>("converge_delta", 0.001);
    declare_parameter<double>("min_match_ratio", 0.30);
    declare_parameter<int>("min_match_points", 30);
    declare_parameter<double>("max_mean_error", 0.10);

    load_params();
    load_map();

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanMatcherLocalizer::odom_cb, this, std::placeholders::_1));
    imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
        imu_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanMatcherLocalizer::imu_cb, this, std::placeholders::_1));
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&ScanMatcherLocalizer::scan_cb, this, std::placeholders::_1));
    pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(pose_topic_, 10);
    if (publish_tf_)
      tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    est_ = start_pose_;            // 初期推定 = スタート姿勢
    RCLCPP_INFO(get_logger(),
        "scan_matcher_localizer started. map=%zu segments, start=(%.2f,%.2f,%.1fdeg)",
        map_.size(), est_.tx, est_.ty, est_.th * 180.0 / M_PI);
  }

private:
  // ── パラメータ読み込み ──
  void load_params() {
    auto sp = get_parameter("start_pose").as_double_array();
    if (sp.size() >= 3) start_pose_ = {sp[0], sp[1], sp[2]};
    lidar_x_ = get_parameter("lidar_x").as_double();
    lidar_y_ = get_parameter("lidar_y").as_double();
    lidar_yaw_ = get_parameter("lidar_yaw").as_double();
    scan_topic_ = get_parameter("scan_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    imu_topic_  = get_parameter("imu_topic").as_string();
    pose_topic_ = get_parameter("pose_topic").as_string();
    publish_tf_ = get_parameter("publish_tf").as_bool();
    map_frame_ = get_parameter("map_frame").as_string();
    odom_frame_ = get_parameter("odom_frame").as_string();
    range_min_ = get_parameter("range_min").as_double();
    range_max_ = get_parameter("range_max").as_double();
    point_skip_ = std::max(1, static_cast<int>(get_parameter("point_skip").as_int()));
    icp_max_iter_ = get_parameter("icp_max_iter").as_int();
    icp_dthre_ = get_parameter("icp_dthre").as_double();
    icp_dthre_end_ = get_parameter("icp_dthre_end").as_double();
    gn_max_iter_ = get_parameter("gn_max_iter").as_int();
    converge_delta_ = get_parameter("converge_delta").as_double();
    min_match_ratio_ = get_parameter("min_match_ratio").as_double();
    min_match_points_ = get_parameter("min_match_points").as_int();
    max_mean_error_ = get_parameter("max_mean_error").as_double();
  }

  void load_map() {
    auto seg = get_parameter("field_map.segments").as_double_array();
    map_.clear();
    for (size_t i = 0; i + 3 < seg.size(); i += 4) {
      Segment s{seg[i], seg[i + 1], seg[i + 2], seg[i + 3], 0, 0, 0, 0, 0, 0};
      s.precompute();
      map_.push_back(s);
    }
    if (map_.empty())
      RCLCPP_WARN(get_logger(), "field_map.segments が空です。マッチングできません。");
  }

  // ── オドメトリ: 最新値を保持 ──
  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const double qz = msg->pose.pose.orientation.z;
    const double qw = msg->pose.pose.orientation.w;
    cur_odom_ = {msg->pose.pose.position.x, msg->pose.pose.position.y,
                 std::atan2(2.0 * qw * qz, 1.0 - 2.0 * qz * qz)};
    have_odom_ = true;
  }

  // ── IMU: ヨー角のみ保持 (yaw は積分のみなので delta で使う) ──
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {
    const double qx = msg->orientation.x, qy = msg->orientation.y;
    const double qz = msg->orientation.z, qw = msg->orientation.w;
    imu_yaw_  = std::atan2(2.0*(qw*qz + qx*qy), 1.0 - 2.0*(qy*qy + qz*qz));
    have_imu_ = true;
  }

  // ── スキャン→base_link座標の点群に変換(LiDAR取付オフセット適用) ──
  std::vector<std::pair<double, double>> scanToBasePoints(const sensor_msgs::msg::LaserScan &scan) {
    std::vector<std::pair<double, double>> pts;
    const double lc = std::cos(lidar_yaw_), ls = std::sin(lidar_yaw_);
    for (size_t i = 0; i < scan.ranges.size(); i += point_skip_) {
      const double r = scan.ranges[i];
      if (!std::isfinite(r) || r < std::max(range_min_, (double)scan.range_min) ||
          r > std::min(range_max_, (double)scan.range_max))
        continue;
      const double a = scan.angle_min + i * scan.angle_increment;
      const double lx = r * std::cos(a), ly = r * std::sin(a);   // ldlidar_link座標
      pts.emplace_back(lc * lx - ls * ly + lidar_x_,             // base_link座標へ
                       ls * lx + lc * ly + lidar_y_);
    }
    return pts;
  }

  // ── メイン: scan-to-known-map ICP ──
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan) {
    if (map_.empty()) return;
    auto pts = scanToBasePoints(*scan);
    if (static_cast<int>(pts.size()) < min_match_points_) return;

    // (1) オドメトリで動きを予測 → ICPの初期値 (ScanMatcher2D の predPose)
    Pose2D pred = est_;
    if (have_odom_ && have_last_odom_) {
      Pose2D motion = relative(cur_odom_, last_odom_);   // 直前→現在の移動量

      // IMUヨーが利用可能なら、エンコーダ由来のΔthをIMUのΔthで上書き
      // (ジャイロ積分はエンコーダより回転精度が高い)
      if (have_imu_ && have_last_imu_yaw_) {
        double dth = imu_yaw_ - last_imu_yaw_;
        dth = std::atan2(std::sin(dth), std::cos(dth));  // ±π に正規化
        motion.th = dth;
      }

      pred = compose(est_, motion);
    }

    // (2) ICP本体 (PoseEstimatorICP): 対応づけ↔GN最適化を収束まで反復
    Pose2D estPose = pred;
    int usedNum = 0;
    double meanErr = std::numeric_limits<double>::infinity();
    for (int it = 0; it < icp_max_iter_; ++it) {
      // 反復が進むほど対応づけゲートを厳しく(外れ値を絞る)
      const double t = (icp_max_iter_ > 1) ? (double)it / (icp_max_iter_ - 1) : 1.0;
      const double dthre = icp_dthre_ + (icp_dthre_end_ - icp_dthre_) * t;

      double maxUpdate = 0.0;
      for (int gn = 0; gn < gn_max_iter_; ++gn) {
        // --- データ対応づけ + ガウスニュートン(点-直線) 1ステップ ---
        Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        const double c = std::cos(estPose.th), s = std::sin(estPose.th);
        int n = 0;
        double errSum = 0.0;
        for (const auto &p : pts) {
          double wx, wy;
          transformPoint(estPose, p.first, p.second, wx, wy);
          // 最近傍の壁線分を探す(DataAssociator)
          const Segment *best = nullptr;
          double bestD = dthre;
          for (const auto &seg : map_) {
            double d = seg.distToPoint(wx, wy);
            if (d < bestD) { bestD = d; best = &seg; }
          }
          if (!best) continue;
          // 残差 = 点-直線の垂直距離 (CostFunctionPD)
          const double r = best->nx * wx + best->ny * wy - best->rho;
          // ヤコビアン dr/d(tx,ty,th)
          const double dwx_dth = -s * p.first - c * p.second;
          const double dwy_dth = c * p.first - s * p.second;
          const double jth = best->nx * dwx_dth + best->ny * dwy_dth;
          Eigen::Vector3d J(best->nx, best->ny, jth);
          H += J * J.transpose();
          b += J * r;
          errSum += std::fabs(r);
          ++n;
        }
        usedNum = n;
        meanErr = (n > 0) ? errSum / n : std::numeric_limits<double>::infinity();
        if (n < min_match_points_) { maxUpdate = 0.0; break; }

        H += 1e-6 * Eigen::Matrix3d::Identity();     // Levenberg減衰(安定化)
        Eigen::Vector3d d = H.ldlt().solve(-b);      // Hδ = -b
        estPose.tx += d(0);
        estPose.ty += d(1);
        estPose.th = std::atan2(std::sin(estPose.th + d(2)), std::cos(estPose.th + d(2)));
        maxUpdate = std::max({std::fabs(d(0)), std::fabs(d(1)), std::fabs(d(2))});
        if (maxUpdate < converge_delta_) break;
      }
      if (maxUpdate < converge_delta_) break;
    }

    // (3) 成否判定 (ScanMatcher2D): 悪ければオドメトリ予測にフォールバック
    const double ratio = (double)usedNum / pts.size();
    const bool ok = (usedNum >= min_match_points_) && (ratio >= min_match_ratio_) &&
                    (meanErr <= max_mean_error_);
    if (!ok) {
      estPose = pred;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "match poor (used=%d ratio=%.2f err=%.3f) -> odom予測使用", usedNum, ratio, meanErr);
    }

    est_ = estPose;
    last_odom_ = cur_odom_;
    have_last_odom_ = have_odom_;
    if (have_imu_) {
      last_imu_yaw_     = imu_yaw_;
      have_last_imu_yaw_ = true;
    }

    publish(estPose, scan->header.stamp, ok, meanErr);
  }

  // ── 推定結果の publish (pose + TF map->odom) ──
  void publish(const Pose2D &p, const rclcpp::Time &stamp, bool ok, double meanErr) {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.pose.pose.position.x = p.tx;
    msg.pose.pose.position.y = p.ty;
    msg.pose.pose.orientation.z = std::sin(p.th / 2.0);
    msg.pose.pose.orientation.w = std::cos(p.th / 2.0);
    const double var = ok ? std::max(1e-4, meanErr * meanErr) : 1.0;  // 失敗時は大きく
    msg.pose.covariance[0] = var;            // x
    msg.pose.covariance[7] = var;            // y
    msg.pose.covariance[35] = ok ? 0.01 : 0.5;  // yaw
    pose_pub_->publish(msg);

    if (tf_broadcaster_ && have_odom_) {
      // map->odom = (map->base) ⊕ (odom->base)⁻¹
      Pose2D map_to_odom = compose(p, inverse(cur_odom_));
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = map_frame_;
      tf.child_frame_id = odom_frame_;
      tf.transform.translation.x = map_to_odom.tx;
      tf.transform.translation.y = map_to_odom.ty;
      tf.transform.rotation.z = std::sin(map_to_odom.th / 2.0);
      tf.transform.rotation.w = std::cos(map_to_odom.th / 2.0);
      tf_broadcaster_->sendTransform(tf);
    }
  }

  // ── メンバ ──
  std::vector<Segment> map_;
  Pose2D start_pose_, est_, cur_odom_, last_odom_;
  bool have_odom_{false}, have_last_odom_{false};

  double imu_yaw_{0.0}, last_imu_yaw_{0.0};
  bool have_imu_{false}, have_last_imu_yaw_{false};

  double lidar_x_, lidar_y_, lidar_yaw_;
  std::string scan_topic_, odom_topic_, imu_topic_, pose_topic_, map_frame_, odom_frame_;
  bool publish_tf_;
  double range_min_, range_max_;
  int point_skip_, icp_max_iter_, gn_max_iter_, min_match_points_;
  double icp_dthre_, icp_dthre_end_, converge_delta_, min_match_ratio_, max_mean_error_;

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScanMatcherLocalizer>());
  rclcpp::shutdown();
  return 0;
}
