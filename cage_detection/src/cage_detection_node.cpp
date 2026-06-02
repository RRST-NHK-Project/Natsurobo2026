/**
 * cage_detection_node.cpp
 *
 * 概要:
 *   Intel RealSense D456 の RGB + 深度を使い、エリア2に固定配置された
 *   7つのカゴ（緑×2、青×5）の3D位置を検出して publishする。
 *   VL53L1X（ToF）の距離値で最終距離を補正する。
 *
 * 検出パイプライン:
 *   RGBフレーム
 *     → HSVマスク（緑/青）
 *     → 輪郭検出 + 矩形フィルタ（面積・アスペクト比）
 *     → 深度フレームで重心の3D座標を取得
 *     → ToF距離で z を補正
 *     → EMA（指数移動平均）でフレーム間平滑化
 *     → ターゲット優先度スコア付き CageArray を publish
 *
 * publishトピック:
 *   /cage_detection/cages      [cage_detection/msg/CageArray]
 *   /cage_detection/target     [cage_detection/msg/Cage]       最優先ターゲット
 *   /cage_detection/debug_image [sensor_msgs/msg/Image]        デバッグ用
 *
 * subscribeトピック:
 *   /camera/color/image_raw         [sensor_msgs/msg/Image]
 *   /camera/depth/image_rect_raw    [sensor_msgs/msg/Image]
 *   /camera/color/camera_info       [sensor_msgs/msg/CameraInfo]
 *   /tof/range                      [sensor_msgs/msg/Range]    VL53L1X
 *   /cage_detection/enable          [std_msgs/msg/Bool]        処理ON/OFF
 *
 * カゴ優先度スコア（高いほど優先）:
 *   最遠青カゴ(スタートゾーン対角) : +150  (C+D+E = 10+10+15pt)
 *   その他青カゴ                   : +100  (C+D    = 10+10pt)
 *   緑カゴ（自チーム）             : +120  (C+D+D  = 10+10+10pt)
 *   占有済みカゴ                   : -999  (選択しない)
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <optional>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"

#include <opencv2/opencv.hpp>

// 自作メッセージ（後述の msg/ に定義）
#include "cage_detection/msg/cage.hpp"
#include "cage_detection/msg/cage_array.hpp"

// ──────────────────────────────────────────────
//  定数
// ──────────────────────────────────────────────
namespace {

// HSVマスク範囲（OpenCV: H=0-179, S=0-255, V=0-255）
// フィールド表面(緑ロンリウム・青ブルーシート)と被るため
// 輝度(V)を絞り、彩度(S)を高めに設定する
const cv::Scalar GREEN_LOW (40,  80, 60);
const cv::Scalar GREEN_HIGH(80, 255, 255);
const cv::Scalar BLUE_LOW  (100, 80, 60);
const cv::Scalar BLUE_HIGH (130, 255, 255);

// カゴID
enum class CageColor : uint8_t { GREEN = 0, BLUE = 1 };

// 優先度スコア
constexpr int SCORE_FARTHEST_BLUE = 150;
constexpr int SCORE_OTHER_BLUE    = 100;
constexpr int SCORE_GREEN         = 120;
constexpr int SCORE_OCCUPIED      = -999;

} // namespace

// ──────────────────────────────────────────────
//  CageCandidate（内部処理用）
// ──────────────────────────────────────────────
struct CageCandidate {
    CageColor   color;
    cv::Rect    bbox;       // 画像座標 [px]
    cv::Point2f centroid;   // 画像重心 [px]
    double      depth_raw;  // D456深度 [m]
    double      depth_tof;  // ToF補正後 [m] (無効=-1)
    geometry_msgs::msg::Point pos3d; // カメラ座標系 [m]
    bool        occupied = false;
    int         priority = 0;
    // EMA平滑化用
    geometry_msgs::msg::Point pos3d_smooth;
    bool        initialized = false;
};

// ──────────────────────────────────────────────
//  ノード本体
// ──────────────────────────────────────────────
class CageDetectionNode : public rclcpp::Node
{
public:
    CageDetectionNode() : Node("cage_detection_node")
    {
        // ── パラメータ ───────────────────────────────
        declare_parameter<double>("min_contour_area",     1500.0);  // px²
        declare_parameter<double>("max_contour_area",   120000.0);
        declare_parameter<double>("min_aspect_ratio",      0.4);
        declare_parameter<double>("max_aspect_ratio",      3.0);
        declare_parameter<double>("ema_alpha",             0.35);   // 平滑化係数
        declare_parameter<double>("depth_roi_size",        5.0);    // 重心周辺ROI [px]
        declare_parameter<double>("tof_weight",            0.6);    // ToF混合比率
        declare_parameter<double>("max_detection_dist",    6.0);    // 最大検出距離 [m]
        declare_parameter<bool>  ("publish_debug_image",   true);
        // フィールド設定: スタートゾーン対角の青カゴ推定Z距離
        // 実フィールドで調整する（エリア2の最遠カゴまでの距離）
        declare_parameter<double>("farthest_cage_z_min",   3.5);    // m

        load_params();

        // ── Publisher ───────────────────────────────
        cages_pub_  = create_publisher<cage_detection::msg::CageArray>(
            "/cage_detection/cages",  10);
        target_pub_ = create_publisher<cage_detection::msg::Cage>(
            "/cage_detection/target", 10);
        if (publish_debug_) {
            debug_pub_ = create_publisher<sensor_msgs::msg::Image>(
                "/cage_detection/debug_image", 10);
        }

        // ── Subscriber ──────────────────────────────
        // RGB
        rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 10,
            std::bind(&CageDetectionNode::rgb_callback, this, std::placeholders::_1));
        // 深度
        depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 10,
            std::bind(&CageDetectionNode::depth_callback, this, std::placeholders::_1));
        // カメラパラメータ
        info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1,
            std::bind(&CageDetectionNode::info_callback, this, std::placeholders::_1));
        // ToF
        tof_sub_ = create_subscription<sensor_msgs::msg::Range>(
            "/tof/range", 10,
            [this](sensor_msgs::msg::Range::SharedPtr msg) {
                latest_tof_dist_ = msg->range;
            });
        // 処理ON/OFF
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/cage_detection/enable", 10,
            [this](std_msgs::msg::Bool::SharedPtr msg) {
                enabled_ = msg->data;
                RCLCPP_INFO(get_logger(), "cage_detection %s", enabled_ ? "ENABLED" : "DISABLED");
            });

        RCLCPP_INFO(get_logger(), "cage_detection_node started.");
    }

private:
    // ── パラメータ読み込み ──────────────────────
    void load_params()
    {
        min_area_       = get_parameter("min_contour_area").as_double();
        max_area_       = get_parameter("max_contour_area").as_double();
        min_aspect_     = get_parameter("min_aspect_ratio").as_double();
        max_aspect_     = get_parameter("max_aspect_ratio").as_double();
        ema_alpha_      = get_parameter("ema_alpha").as_double();
        depth_roi_size_ = get_parameter("depth_roi_size").as_double();
        tof_weight_     = get_parameter("tof_weight").as_double();
        max_dist_       = get_parameter("max_detection_dist").as_double();
        publish_debug_  = get_parameter("publish_debug_image").as_bool();
        farthest_z_min_ = get_parameter("farthest_cage_z_min").as_double();
    }

    // ── カメラ内部パラメータ受信 ────────────────
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (camera_ready_) return;
        fx_ = msg->k[0];
        fy_ = msg->k[4];
        cx_ = msg->k[2];
        cy_ = msg->k[5];
        camera_ready_ = true;
        RCLCPP_INFO(get_logger(), "Camera intrinsics: fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
            fx_, fy_, cx_, cy_);
    }

    // ── 深度フレーム受信（保持のみ）───────────────
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        latest_depth_ = msg;
    }

    // ── RGBフレーム受信 → メイン処理 ────────────
    void rgb_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!enabled_ || !camera_ready_ || !latest_depth_) return;

        // cv::Mat に変換
        cv::Mat rgb;
        try {
            rgb = cv_bridge::toCvShare(msg, "bgr8")->image;
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "cv_bridge: %s", e.what());
            return;
        }

        cv::Mat depth;
        try {
            auto depth_msg = cv_bridge::toCvShare(latest_depth_, "16UC1");
            depth_msg->image.convertTo(depth, CV_32FC1, 1e-3); // mm → m
        } catch (const cv_bridge::Exception& e) {
            RCLCPP_WARN(get_logger(), "depth cv_bridge: %s", e.what());
            return;
        }

        // ── 検出 ──────────────────────────────────
        auto candidates = detect_cages(rgb, depth);

        // ── 3D位置計算 + ToF補正 ──────────────────
        for (auto& c : candidates) {
            compute_3d(c, depth);
            apply_tof_correction(c);
        }

        // ── EMA平滑化 ─────────────────────────────
        update_tracks(candidates);

        // ── 優先度スコア付け ──────────────────────
        assign_priority(candidates);

        // ── publish ───────────────────────────────
        publish_results(candidates, msg->header);

        // ── デバッグ描画 ──────────────────────────
        if (publish_debug_) {
            draw_debug(rgb, candidates);
            auto debug_msg = cv_bridge::CvImage(msg->header, "bgr8", rgb).toImageMsg();
            debug_pub_->publish(*debug_msg);
        }
    }

    // ── HSV色検出 + 輪郭フィルタ ─────────────────
    std::vector<CageCandidate> detect_cages(
        const cv::Mat& rgb, const cv::Mat& /*depth*/) const
    {
        cv::Mat hsv;
        cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

        std::vector<CageCandidate> result;

        // フィールド床面（緑ロンリウム・青ブルーシート）との誤検知対策:
        // 画像下半分のエリアで検出を行う（石倉上からの見下ろし視点）
        // 必要に応じて ROI を調整
        const int roi_y = rgb.rows / 3;  // 上1/3はスキップ
        cv::Rect roi_rect(0, roi_y, rgb.cols, rgb.rows - roi_y);
        cv::Mat hsv_roi = hsv(roi_rect);

        for (int color_idx = 0; color_idx < 2; ++color_idx) {
            const cv::Scalar& low  = (color_idx == 0) ? GREEN_LOW  : BLUE_LOW;
            const cv::Scalar& high = (color_idx == 0) ? GREEN_HIGH : BLUE_HIGH;
            const CageColor   col  = (color_idx == 0) ? CageColor::GREEN : CageColor::BLUE;

            cv::Mat mask;
            cv::inRange(hsv_roi, low, high, mask);

            // ノイズ除去
            cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, {5, 5});
            cv::morphologyEx(mask, mask, cv::MORPH_OPEN,  kernel);
            cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

            // 輪郭検出
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            for (const auto& cnt : contours) {
                const double area = cv::contourArea(cnt);
                if (area < min_area_ || area > max_area_) continue;

                cv::Rect bbox = cv::boundingRect(cnt);
                // ROIオフセット補正
                bbox.y += roi_y;

                const double aspect = static_cast<double>(bbox.width) / bbox.height;
                if (aspect < min_aspect_ || aspect > max_aspect_) continue;

                // 重心
                auto M = cv::moments(cnt);
                if (std::abs(M.m00) < 1e-6) continue;
                cv::Point2f centroid(
                    static_cast<float>(M.m10 / M.m00),
                    static_cast<float>(M.m01 / M.m00) + roi_y);

                CageCandidate c;
                c.color    = col;
                c.bbox     = bbox;
                c.centroid = centroid;
                result.push_back(c);
            }
        }
        return result;
    }

    // ── 3D位置計算（ピンホールモデル） ───────────
    void compute_3d(CageCandidate& c, const cv::Mat& depth) const
    {
        // 重心周辺の深度中央値を使う（外れ値に強い）
        const int r  = static_cast<int>(depth_roi_size_);
        const int px = static_cast<int>(c.centroid.x);
        const int py = static_cast<int>(c.centroid.y);

        const int x0 = std::max(0, px - r);
        const int y0 = std::max(0, py - r);
        const int x1 = std::min(depth.cols - 1, px + r);
        const int y1 = std::min(depth.rows - 1, py + r);

        std::vector<float> vals;
        vals.reserve((x1-x0+1)*(y1-y0+1));
        for (int y = y0; y <= y1; ++y) {
            for (int x = x0; x <= x1; ++x) {
                float d = depth.at<float>(y, x);
                if (d > 0.1f && d < static_cast<float>(max_dist_)) {
                    vals.push_back(d);
                }
            }
        }

        if (vals.empty()) {
            c.depth_raw = -1.0;
            return;
        }

        // 中央値
        std::nth_element(vals.begin(), vals.begin() + vals.size()/2, vals.end());
        c.depth_raw = vals[vals.size()/2];

        // ピンホール逆投影
        const double z = c.depth_raw;
        c.pos3d.x = (c.centroid.x - cx_) * z / fx_;
        c.pos3d.y = (c.centroid.y - cy_) * z / fy_;
        c.pos3d.z = z;
    }

    // ── ToF補正 ──────────────────────────────────
    // ToFはロボット正面の1点距離を返す。カゴが視野中央にあるとき
    // depth_raw と tof_dist の加重平均で z を補正する。
    void apply_tof_correction(CageCandidate& c) const
    {
        if (latest_tof_dist_ < 0.1 || c.depth_raw < 0.1) {
            c.depth_tof = c.depth_raw;
            return;
        }
        // 視野中央付近のカゴにのみ適用（重心が画像中心から±15%以内）
        const float cx_ratio = std::abs(c.centroid.x - static_cast<float>(cx_))
                               / static_cast<float>(cx_);
        if (cx_ratio > 0.15f) {
            c.depth_tof = c.depth_raw;
            return;
        }

        c.depth_tof = tof_weight_     * latest_tof_dist_
                    + (1.0 - tof_weight_) * c.depth_raw;

        // z補正に合わせてpos3dも更新
        if (c.depth_raw > 1e-3) {
            const double scale = c.depth_tof / c.depth_raw;
            c.pos3d.x *= scale;
            c.pos3d.y *= scale;
            c.pos3d.z  = c.depth_tof;
        }
    }

    // ── EMA トラッキング ──────────────────────────
    // フレーム間でカゴIDを維持する簡易トラッカー
    // 同色カゴを画像座標で最近傍マッチング
    void update_tracks(std::vector<CageCandidate>& candidates)
    {
        for (auto& c : candidates) {
            if (c.depth_raw < 0) continue;

            // トラックIDのキー: 色 × グリッド位置（簡易）
            // カゴは固定配置なので、X座標のビンでIDを割り当て
            const int color_key = static_cast<int>(c.color);
            const int x_bin     = static_cast<int>(c.pos3d.x / 0.3); // 30cm刻み
            const int track_key = color_key * 1000 + x_bin;

            auto it = tracks_.find(track_key);
            if (it == tracks_.end()) {
                c.pos3d_smooth = c.pos3d;
                c.initialized  = true;
                tracks_[track_key] = c;
            } else {
                auto& prev = it->second;
                c.pos3d_smooth.x = (1.0 - ema_alpha_) * prev.pos3d_smooth.x
                                 + ema_alpha_ * c.pos3d.x;
                c.pos3d_smooth.y = (1.0 - ema_alpha_) * prev.pos3d_smooth.y
                                 + ema_alpha_ * c.pos3d.y;
                c.pos3d_smooth.z = (1.0 - ema_alpha_) * prev.pos3d_smooth.z
                                 + ema_alpha_ * c.pos3d.z;
                c.initialized    = true;
                it->second = c;
            }
        }
    }

    // ── 優先度スコア ──────────────────────────────
    void assign_priority(std::vector<CageCandidate>& candidates) const
    {
        for (auto& c : candidates) {
            if (c.occupied) {
                c.priority = SCORE_OCCUPIED;
                continue;
            }
            if (c.color == CageColor::GREEN) {
                c.priority = SCORE_GREEN;
            } else {
                // スタートゾーン対角の最遠青カゴ: z が大きい（遠い）ものを優先
                c.priority = (c.pos3d_smooth.z >= farthest_z_min_)
                           ? SCORE_FARTHEST_BLUE
                           : SCORE_OTHER_BLUE;
            }
        }
        // スコア降順でソート
        std::sort(candidates.begin(), candidates.end(),
            [](const CageCandidate& a, const CageCandidate& b){
                return a.priority > b.priority;
            });
    }

    // ── メッセージ変換 + publish ──────────────────
    void publish_results(
        const std::vector<CageCandidate>& candidates,
        const std_msgs::msg::Header& header)
    {
        cage_detection::msg::CageArray arr;
        arr.header = header;

        for (const auto& c : candidates) {
            if (!c.initialized || c.depth_raw < 0) continue;

            cage_detection::msg::Cage cage;
            cage.header   = header;
            cage.color    = static_cast<uint8_t>(c.color);
            cage.position = c.pos3d_smooth;
            cage.priority = c.priority;
            cage.occupied = c.occupied;
            cage.distance = c.pos3d_smooth.z;
            arr.cages.push_back(cage);
        }

        cages_pub_->publish(arr);

        // 最優先ターゲット（スコア最大・occupied除外）
        for (const auto& c : arr.cages) {
            if (c.priority > SCORE_OCCUPIED) {
                target_pub_->publish(c);
                break;
            }
        }
    }

    // ── デバッグ描画 ──────────────────────────────
    void draw_debug(cv::Mat& img,
                    const std::vector<CageCandidate>& candidates) const
    {
        for (const auto& c : candidates) {
            const cv::Scalar col = (c.color == CageColor::GREEN)
                                 ? cv::Scalar(0, 255, 0)
                                 : cv::Scalar(255, 100, 0);
            cv::rectangle(img, c.bbox, col, 2);
            cv::circle(img, cv::Point(
                static_cast<int>(c.centroid.x),
                static_cast<int>(c.centroid.y)), 5, col, -1);

            // 距離・優先度テキスト
            if (c.depth_raw > 0) {
                char buf[64];
                std::snprintf(buf, sizeof(buf),
                    "z=%.2fm p=%d", c.pos3d_smooth.z, c.priority);
                cv::putText(img, buf,
                    cv::Point(c.bbox.x, c.bbox.y - 6),
                    cv::FONT_HERSHEY_SIMPLEX, 0.45, col, 1);
            }
        }
    }

    // ── メンバ変数 ──────────────────────────────
    // パラメータ
    double min_area_, max_area_;
    double min_aspect_, max_aspect_;
    double ema_alpha_;
    double depth_roi_size_;
    double tof_weight_;
    double max_dist_;
    bool   publish_debug_;
    double farthest_z_min_;

    // カメラ内部パラメータ
    double fx_ = 0, fy_ = 0, cx_ = 0, cy_ = 0;
    bool camera_ready_ = false;

    // 状態
    bool   enabled_          = true;
    double latest_tof_dist_  = -1.0;
    sensor_msgs::msg::Image::SharedPtr latest_depth_;

    // EMAトラッキング（key: 色×X位置ビン）
    std::unordered_map<int, CageCandidate> tracks_;

    // ROS
    rclcpp::Publisher<cage_detection::msg::CageArray>::SharedPtr cages_pub_;
    rclcpp::Publisher<cage_detection::msg::Cage>::SharedPtr      target_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr        debug_pub_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      rgb_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr      depth_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr      tof_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr          enable_sub_;
};

// ──────────────────────────────────────────────
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CageDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
