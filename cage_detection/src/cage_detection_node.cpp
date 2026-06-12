/**
 * cage_detection_node.cpp  (v3 — 固定座標版)
 *
 * カゴ位置をYAMLに固定値として定義し、
 * wall_detection の角度・距離から自己位置を推定して
 * カゴのカメラ座標系3D位置を計算してpublishする。
 *
 * 座標系定義（フィールド座標系）:
 *   原点: スタートゾーン中心
 *   X: スタートゾーンから見てフィールド奥方向（+）
 *   Y: 左方向（+）
 *   単位: [m]
 *
 * 自己位置推定:
 *   wall_distance = 石倉側面からロボットまでの距離 [m]
 *   wall_angle    = 石倉側面に対するロボットの偏角 [rad]
 *   → ロボットのフィールド座標 (robot_x, robot_y, robot_yaw) を推定
 *   → 各カゴ座標をカメラ座標系（前方=z）に変換
 *
 * publishトピック:
 *   /cage_detection/cages   [cage_detection/msg/CageArray]
 *   /cage_detection/target  [cage_detection/msg/Cage]  最優先ターゲット
 *
 * subscribeトピック:
 *   /wall_detection/angle    [std_msgs/Float64]
 *   /wall_detection/distance [std_msgs/Float64]
 *   /cage_detection/enable   [std_msgs/Bool]
 */

#include <cmath>
#include <vector>
#include <algorithm>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"

#include "cage_detection/msg/cage.hpp"
#include "cage_detection/msg/cage_array.hpp"

// 優先度スコア
constexpr int SCORE_FARTHEST_BLUE = 150;
constexpr int SCORE_OTHER_BLUE    = 100;
constexpr int SCORE_GREEN         = 120;
constexpr int SCORE_OCCUPIED      = -999;

// ──────────────────────────────────────────────
//  カゴの定義
// ──────────────────────────────────────────────
struct CageDef {
    std::string id;      // "green_0", "blue_0" 等
    uint8_t     color;   // 0=GREEN, 1=BLUE
    double      field_x; // フィールド座標X [m]
    double      field_y; // フィールド座標Y [m]
    bool        is_farthest; // スタートゾーン対角の青カゴか
};

// ──────────────────────────────────────────────
class CageDetectionNode : public rclcpp::Node
{
public:
    CageDetectionNode() : Node("cage_detection_node")
    {
        // ── カゴ固定座標パラメータ ────────────────────
        // 単位: [m]、フィールド座標系
        // !! 実フィールドで計測してここを更新すること !!
        //
        // フィールド寸法（ルールブック図4より）:
        //   全体: 10800×10800mm
        //   エリア2(ブルーシート): 3333.5×3333.5mm
        //   エリア2中心はフィールド中心
        //
        // 原点: スタートゾーン中心
        // スタートゾーン→エリア2端まで: (10800/2 - 3333.5/2) ≈ 3733mm = 3.733m
        //
        // 図6のカゴ配置（エリア2内、7つ）:
        //   行1(奥): 左、右          → 2つ
        //   行2(中): 左、中、右      → 3つ
        //   行3(手前): 左、右        → 2つ
        // ※ 実際の座標は実フィールド計測で確定すること

        // 緑カゴ2つ
        declare_parameter<double>("cage.green_0.x", 4.20);
        declare_parameter<double>("cage.green_0.y", 0.55);
        declare_parameter<double>("cage.green_1.x", 4.20);
        declare_parameter<double>("cage.green_1.y",-0.55);

        // 青カゴ5つ
        declare_parameter<double>("cage.blue_0.x", 5.40);  // 最奥左
        declare_parameter<double>("cage.blue_0.y", 1.10);
        declare_parameter<double>("cage.blue_1.x", 5.40);  // 最奥右（最遠カゴ候補）
        declare_parameter<double>("cage.blue_1.y",-1.10);
        declare_parameter<double>("cage.blue_2.x", 4.80);  // 中左
        declare_parameter<double>("cage.blue_2.y", 0.0);
        declare_parameter<double>("cage.blue_3.x", 3.73);  // 手前左
        declare_parameter<double>("cage.blue_3.y", 1.10);
        declare_parameter<double>("cage.blue_4.x", 3.73);  // 手前右
        declare_parameter<double>("cage.blue_4.y",-1.10);

        // 石倉の位置（ロボットが乗る場所）
        // スタートゾーン端からエリア3まで
        declare_parameter<double>("ishikura_x", 3.20);  // フィールドX [m]
        declare_parameter<double>("ishikura_y", 0.0);

        // スタートゾーン対角の青カゴID
        declare_parameter<std::string>("farthest_blue_id", "blue_1");

        // 制御ループ周期
        declare_parameter<double>("publish_rate_hz", 10.0);

        load_params();

        // ── Publisher ───────────────────────────────
        cages_pub_  = create_publisher<cage_detection::msg::CageArray>(
            "/cage_detection/cages", 10);
        target_pub_ = create_publisher<cage_detection::msg::Cage>(
            "/cage_detection/target", 10);

        // ── Subscriber ──────────────────────────────
        angle_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){ wall_angle_ = m->data; });
        dist_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/distance", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){ wall_dist_ = m->data; });
        enable_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/cage_detection/enable", 10,
            [this](std_msgs::msg::Bool::SharedPtr m){ enabled_ = m->data; });

        // ── タイマー ─────────────────────────────────
        const int ms = static_cast<int>(1000.0 / get_parameter("publish_rate_hz").as_double());
        timer_ = create_wall_timer(
            std::chrono::milliseconds(ms),
            std::bind(&CageDetectionNode::publish_loop, this));

        RCLCPP_INFO(get_logger(), "cage_detection_node (v3 固定座標版) started. %zu cages loaded.",
            cages_.size());
    }

private:
    void load_params()
    {
        const std::string farthest_id =
            get_parameter("farthest_blue_id").as_string();

        const double ishi_x = get_parameter("ishikura_x").as_double();
        const double ishi_y = get_parameter("ishikura_y").as_double();
        ishikura_pos_ = {ishi_x, ishi_y};

        // カゴ定義をロード
        cages_.clear();

        auto add = [&](const std::string& id, uint8_t color) {
            CageDef def;
            def.id    = id;
            def.color = color;
            def.field_x   = get_parameter("cage." + id + ".x").as_double();
            def.field_y   = get_parameter("cage." + id + ".y").as_double();
            def.is_farthest = (id == farthest_id);
            cages_.push_back(def);
        };

        add("green_0", 0);
        add("green_1", 0);
        add("blue_0",  1);
        add("blue_1",  1);
        add("blue_2",  1);
        add("blue_3",  1);
        add("blue_4",  1);
    }

    // ── メイン配信ループ ──────────────────────────
    void publish_loop()
    {
        if (!enabled_) return;

        // ── 自己位置推定 ─────────────────────────────
        // 石倉の上に乗った状態で wall_detection を使う。
        // 石倉側面（X方向の壁）に正対しているとき:
        //   wall_distance = ロボット〜石倉側面の距離
        //   wall_angle    = 壁に対する偏角（正対なら≈0）
        //
        // ロボットのフィールドX座標:
        //   robot_x = ishikura_x + robot_depth_from_ishikura
        //   (石倉側面からの距離は shooting_control で使うため、ここでは
        //    石倉XYを原点としてカゴ相対座標に変換する)

        const double robot_x   = ishikura_pos_.first;
        const double robot_y   = ishikura_pos_.second;
        const double robot_yaw = wall_angle_; // 壁偏角 = ロボットのヨー誤差

        // ── 各カゴのカメラ座標を計算 ─────────────────
        // フィールド座標系からカメラ座標系（前方=z, 右=x, 下=y）へ変換
        // カメラはロボット前方を向いているとして:
        //   field_dx = cage_x - robot_x
        //   field_dy = cage_y - robot_y
        //   camera_z = field_dx * cos(yaw) + field_dy * sin(yaw)
        //   camera_x = -field_dx * sin(yaw) + field_dy * cos(yaw)

        auto header = std_msgs::msg::Header();
        header.stamp    = now();
        header.frame_id = "camera_color_optical_frame";

        cage_detection::msg::CageArray arr;
        arr.header = header;

        for (const auto& def : cages_) {
            const double dx = def.field_x - robot_x;
            const double dy = def.field_y - robot_y;

            const double cam_z =  dx * std::cos(robot_yaw) + dy * std::sin(robot_yaw);
            const double cam_x = -dx * std::sin(robot_yaw) + dy * std::cos(robot_yaw);

            // 後方のカゴは無視
            if (cam_z < 0.1) continue;

            cage_detection::msg::Cage cage;
            cage.header       = header;
            cage.color        = def.color;
            cage.position.x   = cam_x;
            cage.position.y   = 0.0;  // 高さ差はshooting_controlで処理
            cage.position.z   = cam_z;
            cage.distance     = cam_z;
            cage.occupied     = false;

            // 優先度スコア
            if (def.color == 0) {               // GREEN
                cage.priority = SCORE_GREEN;
            } else if (def.is_farthest) {       // 最遠青
                cage.priority = SCORE_FARTHEST_BLUE;
            } else {                            // その他青
                cage.priority = SCORE_OTHER_BLUE;
            }

            arr.cages.push_back(cage);
        }

        // 優先度降順ソート
        std::sort(arr.cages.begin(), arr.cages.end(),
            [](const auto& a, const auto& b){ return a.priority > b.priority; });

        cages_pub_->publish(arr);

        // 最優先ターゲット
        if (!arr.cages.empty()) {
            target_pub_->publish(arr.cages.front());
        }
    }

    // ── メンバ変数 ──────────────────────────────
    std::vector<CageDef> cages_;
    std::pair<double,double> ishikura_pos_;

    double wall_angle_ = 0.0;
    double wall_dist_  = 0.0;
    bool   enabled_    = true;

    rclcpp::Publisher<cage_detection::msg::CageArray>::SharedPtr cages_pub_;
    rclcpp::Publisher<cage_detection::msg::Cage>::SharedPtr      target_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      angle_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr      dist_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr         enable_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CageDetectionNode>());
    rclcpp::shutdown();
    return 0;
}
