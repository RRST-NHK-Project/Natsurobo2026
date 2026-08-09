/*
 * natsu_auto_node.cpp
 *
 * 夏ロボ2026 自律シーケンスFSM(有限オートマトン遷移図)
 * 
 * subscribe:
 *   /auto/start            [std_msgs/Bool]      シーケンス開始(GUI)
 *   /auto/collect_done     [std_msgs/Bool]      手動回収完了(GUI)
 *   /auto/abort            [std_msgs/Bool]      緊急中断(GUI/PS4)
 *   /auto/force_state      [std_msgs/String]    任意状態へ強制遷移(GUI, 動作完了を待たない)
 *   /wall_detection/angle [std_msgs/Float64]   壁偏角[rad]
 *   /wall_detection/distance [std_msgs/Float64] 壁距離[m]
 *   /localization/pose    [geometry_msgs/PoseWithCovarianceStamped] 自己位置
 *   /climb/done           [std_msgs/Bool]      昇降完了
 *
 * publish:
 *   /cmd_vel              [geometry_msgs/Twist]  走行指令(→zakiomni)
 *   /climb/start          [std_msgs/Bool]        昇降開始
 *   /collect/start        [std_msgs/Bool]        回収開始(手動運用時は未使用可)
 *   /shooter/fire_request [std_msgs/Bool]        射出要求
 *   /auto/state            [std_msgs/String]      現在状態(GUI表示)
 *   /auto/arrived          [std_msgs/Bool]        定位置到達(GUI「移動完了！」)
 *   /auto/arbitration      [std_msgs/String]      調停指示 "auto"/"manual"
 */

#include <cmath>
#include <string>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

// ──────────────────────────────────────────────
enum class State {
    IDLE,
    DETECT_STEP,
    ALIGN,
    CLIMB,
    MOVE_TO_HOME,
    MANUAL_COLLECT,
    MOVE_TO_SHOOT,
    FIRE,
    DONE,
    ABORTED
};

static const char* state_name(State s)
{
    switch (s) {
        case State::IDLE:           return "IDLE";
        case State::DETECT_STEP:    return "DETECT_STEP";
        case State::ALIGN:          return "ALIGN";
        case State::CLIMB:          return "CLIMB";
        case State::MOVE_TO_HOME:   return "MOVE_TO_HOME";
        case State::MANUAL_COLLECT: return "MANUAL_COLLECT";
        case State::MOVE_TO_SHOOT:  return "MOVE_TO_SHOOT";
        case State::FIRE:           return "FIRE";
        case State::DONE:           return "DONE";
        case State::ABORTED:        return "ABORTED";
    }
    return "UNKNOWN";
}

// 状態名(文字列) → State。未知の名前なら std::nullopt。
static std::optional<State> state_from_name(const std::string& n)
{
    if (n == "IDLE")           return State::IDLE;
    if (n == "DETECT_STEP")    return State::DETECT_STEP;
    if (n == "ALIGN")          return State::ALIGN;
    if (n == "CLIMB")          return State::CLIMB;
    if (n == "MOVE_TO_HOME")   return State::MOVE_TO_HOME;
    if (n == "MANUAL_COLLECT") return State::MANUAL_COLLECT;
    if (n == "MOVE_TO_SHOOT")  return State::MOVE_TO_SHOOT;
    if (n == "FIRE")           return State::FIRE;
    if (n == "DONE")           return State::DONE;
    if (n == "ABORTED")        return State::ABORTED;
    return std::nullopt;
}

// --- 状態ごとのフラグ束 (Google C++ Style Guide 準拠) ---
// データを束ねるだけ(不変条件なし)なので struct を使う。
// struct のメンバは末尾アンダースコアを付けない(class のデータメンバとは異なる)。

// CLIMB 状態だけが使うフラグ
struct ClimbState {
    bool started = false;                       // /climb/start を発行済みか
    bool done    = false;                       // /climb/done を受信したか
    rclcpp::Time start_time{0,0,RCL_ROS_TIME};  // 昇降開始時刻(タイムアウト用)
};

// MANUAL_COLLECT 状態だけが使うフラグ
struct CollectState {
    bool handover = false;                      // 調停を手動に明け渡し済みか
    bool done     = false;                      // /auto/collect_done を受信したか
};

// FIRE 状態だけが使うフラグ
struct FireState {
    bool started = false;                       // 射出要求を開始済みか
    rclcpp::Time start_time{0,0,RCL_ROS_TIME};  // 射出開始時刻
};

// ──────────────────────────────────────────────
class NatsuAutoNode : public rclcpp::Node
{
public:
    NatsuAutoNode() : Node("natsu_auto_node")
    {
        //　パラメータ（全て仮お気なので実測すること）
        // 段差検知: 壁距離がこれ以下なら石倉前とみなす [m]
        declare_parameter<double>("step_detect_distance", 0.30);
        // 石倉に向かって詰めるときの前進速度 [m/s]
        declare_parameter<double>("approach_v", 0.15);
        // 前進の打ち切り [s]。壁検知が出ない/閾値に届かない場合の走行距離上限を決める
        declare_parameter<double>("approach_timeout", 8.0);
        // 壁検知値をこの秒数より古ければ「無効」とみなす [s]
        declare_parameter<double>("wall_data_timeout", 0.5);
        // false にすると壁検知を待たず approach_timeout だけで次へ進む
        declare_parameter<bool>("use_wall_detection", true);

        // 平行判定: 壁偏角の許容 [rad]
        declare_parameter<double>("align_angle_tol", 0.05);
        // 平行旋回の角速度ゲイン
        declare_parameter<double>("align_kp", 1.5);
        declare_parameter<double>("align_omega_max", 1.0);   // [rad/s]
        // 平行が出せないまま回り続けるのを防ぐ打ち切り [s]
        declare_parameter<double>("align_timeout", 6.0);

        // 定位置(HOME)座標 [m, m, rad] (map系、仮置き)
        declare_parameter<double>("home_x", 3.20);
        declare_parameter<double>("home_y", 0.30);
        declare_parameter<double>("home_yaw", 0.0);

        // 射出位置(SHOOT)座標 [m, m, rad] (仮置き)
        declare_parameter<double>("shoot_x", 3.20);
        declare_parameter<double>("shoot_y", -0.20);
        declare_parameter<double>("shoot_yaw", 0.0);

        // 位置到達判定 [m] / [rad]
        declare_parameter<double>("pos_tol", 0.05);
        declare_parameter<double>("yaw_tol", 0.05);

        // 移動制御ゲイン
        declare_parameter<double>("move_kp", 1.2);
        declare_parameter<double>("move_v_max", 0.5);
        declare_parameter<double>("move_kp_yaw", 1.5);
        declare_parameter<double>("move_omega_max", 1.0);

        // 昇降完了のタイムアウト保険 [s]
        declare_parameter<double>("climb_timeout", 20.0);
        
        // 射出パルスの長さ [s]
        declare_parameter<double>("fire_hold", 1.0);

        // 制御周期 [Hz]
        declare_parameter<double>("rate_hz", 20.0);

        load_params();

        // ── Publisher ───────────────────────────────
        cmd_vel_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        climb_pub_     = create_publisher<std_msgs::msg::Bool>("/climb/start", 10);
        collect_pub_   = create_publisher<std_msgs::msg::Bool>("/collect/start", 10);
        fire_pub_      = create_publisher<std_msgs::msg::Bool>("/shooter/fire_request", 10);
        state_pub_     = create_publisher<std_msgs::msg::String>("/auto/state", 10);
        arrived_pub_   = create_publisher<std_msgs::msg::Bool>("/auto/arrived", 10);
        arb_pub_       = create_publisher<std_msgs::msg::String>("/auto/arbitration", 10);

        // ── Subscriber ──────────────────────────────
        start_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/auto/start", 10,
            [this](std_msgs::msg::Bool::SharedPtr m){ if (m->data) on_start(); });

        collect_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/auto/collect_done", 10,
            [this](std_msgs::msg::Bool::SharedPtr m){ if (m->data) collect_.done = true; });

        abort_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/auto/abort", 10,
            [this](std_msgs::msg::Bool::SharedPtr m){ if (m->data) on_abort(); });

        // GUIからの強制ステート遷移(動作完了を待たない。状態名の文字列)
        force_state_sub_ = create_subscription<std_msgs::msg::String>(
            "/auto/force_state", 10,
            [this](std_msgs::msg::String::SharedPtr m){ on_force_state(m->data); });

        angle_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/angle", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){
                wall_angle_ = m->data; wall_angle_stamp_ = now(); });

        dist_sub_ = create_subscription<std_msgs::msg::Float64>(
            "/wall_detection/distance", 10,
            [this](std_msgs::msg::Float64::SharedPtr m){
                wall_dist_ = m->data; wall_dist_stamp_ = now(); });

        pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/localization/pose", 10,
            std::bind(&NatsuAutoNode::pose_cb, this, std::placeholders::_1));

        climb_done_sub_ = create_subscription<std_msgs::msg::Bool>(
            "/climb/done", 10,
            [this](std_msgs::msg::Bool::SharedPtr m){ if (m->data) climb_.done = true; });

        // ── 制御ループ ───────────────────────────────
        const int ms = static_cast<int>(1000.0 / rate_hz_);
        timer_ = create_wall_timer(
            std::chrono::milliseconds(ms),
            std::bind(&NatsuAutoNode::loop, this));

        state_entered_ = now();
        RCLCPP_INFO(get_logger(), "natsu_auto_node started. state=IDLE");
        publish_state();
        publish_arbitration("manual"); // IDLE=手動待機。autoは自動走行状態に入ってから
    }

private:
    void load_params()
    {
        step_dist_     = get_parameter("step_detect_distance").as_double();
        approach_v_    = get_parameter("approach_v").as_double();
        approach_timeout_ = get_parameter("approach_timeout").as_double();
        wall_data_timeout_ = get_parameter("wall_data_timeout").as_double();
        use_wall_detection_ = get_parameter("use_wall_detection").as_bool();
        align_tol_     = get_parameter("align_angle_tol").as_double();
        align_kp_      = get_parameter("align_kp").as_double();
        align_omega_max_ = get_parameter("align_omega_max").as_double();
        align_timeout_ = get_parameter("align_timeout").as_double();
        home_x_        = get_parameter("home_x").as_double();
        home_y_        = get_parameter("home_y").as_double();
        home_yaw_      = get_parameter("home_yaw").as_double();
        shoot_x_       = get_parameter("shoot_x").as_double();
        shoot_y_       = get_parameter("shoot_y").as_double();
        shoot_yaw_     = get_parameter("shoot_yaw").as_double();
        pos_tol_       = get_parameter("pos_tol").as_double();
        yaw_tol_       = get_parameter("yaw_tol").as_double();
        move_kp_       = get_parameter("move_kp").as_double();
        move_v_max_    = get_parameter("move_v_max").as_double();
        move_kp_yaw_   = get_parameter("move_kp_yaw").as_double();
        move_omega_max_= get_parameter("move_omega_max").as_double();
        climb_timeout_ = get_parameter("climb_timeout").as_double();
        fire_hold_     = get_parameter("fire_hold").as_double();
        rate_hz_       = get_parameter("rate_hz").as_double();
    }

    // ── イベント ────────────────────────────────
    void on_start()
    {
        if (state_ == State::IDLE || state_ == State::DONE || state_ == State::ABORTED) {
            transit(State::DETECT_STEP);
        }
    }

    void on_abort()
    {
        stop_robot();
        transit(State::ABORTED);
        publish_arbitration("manual");  // 中断時は手動へ明け渡す
        RCLCPP_WARN(get_logger(), "ABORTED by request.");
    }

    // 動作完了を待たず、GUIから任意の状態へ強制遷移(デバッグ/手動運用)。
    // 誤操作防止のロックはGUI側で担保。ここは受けたら即遷移する。
    void on_force_state(const std::string& name)
    {
        auto s = state_from_name(name);
        if (!s) {
            RCLCPP_WARN(get_logger(), "force_state: 未知の状態名 '%s' を無視", name.c_str());
            return;
        }
        stop_robot();  // 現動作の指令を止めてから遷移(飛び出し防止)
        RCLCPP_WARN(get_logger(), "force_state: %s → %s へ強制遷移",
                    state_name(state_), state_name(*s));
        transit(*s);
        // 自動走行を伴う状態へ飛ぶなら制御権をautoへ、手動回収へ飛ぶならmanualへ
        publish_arbitration(*s == State::MANUAL_COLLECT ? "manual" : "auto");
    }

    void pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr m)
    {
        cur_x_ = m->pose.pose.position.x;
        cur_y_ = m->pose.pose.position.y;
        const auto& q = m->pose.pose.orientation;
        cur_yaw_ = std::atan2(2.0*(q.w*q.z + q.x*q.y),
                              1.0 - 2.0*(q.y*q.y + q.z*q.z));
        pose_ok_ = true;
    }

    // ── メインループ ────────────────────────────
    void loop()
    {
        switch (state_) {
            case State::IDLE:           break;  // /auto/start 待ち
            case State::DETECT_STEP:    do_detect_step(); break;
            case State::ALIGN:          do_align();       break;
            case State::CLIMB:          do_climb();       break;
            case State::MOVE_TO_HOME:   do_move(home_x_, home_y_, home_yaw_, true);  break;
            case State::MANUAL_COLLECT: do_manual_collect(); break;
            case State::MOVE_TO_SHOOT:  do_move(shoot_x_, shoot_y_, shoot_yaw_, false); break;
            case State::FIRE:           do_fire();        break;
            case State::DONE:           break;
            case State::ABORTED:        break;
        }
        publish_state();

        // 調停は毎周期publishする（遷移時の1回だけだと、後から起動した
        // zakiomniが現在モードを受け取れず、cmd_velが永遠に無視されるため）
        publish_arbitration(state_is_auto(state_) ? "auto" : "manual");
    }

    // 自動走行側が機体を持つべき状態か（それ以外はjoy専属＝手動）
    static bool state_is_auto(State s)
    {
        return s == State::DETECT_STEP || s == State::ALIGN ||
               s == State::CLIMB       || s == State::MOVE_TO_HOME ||
               s == State::MOVE_TO_SHOOT || s == State::FIRE;
    }

    // ── 各ステート処理 ──────────────────────────

    // 段差検知: 石倉に向かって前進し、壁距離が閾値以下になったら停止
    void do_detect_step()
    {
        const double elapsed = (now() - state_entered_).seconds();

        if (use_wall_detection_ && wall_dist_fresh() && wall_dist_ <= step_dist_) {
            RCLCPP_INFO(get_logger(), "段差検知: dist=%.2fm (%.1fs)", wall_dist_, elapsed);
            stop_robot();
            transit(State::ALIGN);
            return;
        }

        // 壁検知が出ない/閾値に届かないまま走り続けないための打ち切り。
        // approach_v × approach_timeout がそのまま前進距離の上限になる。
        if (elapsed > approach_timeout_) {
            const std::string d = wall_dist_fresh()
                                ? std::to_string(wall_dist_) : std::string("無効");
            RCLCPP_WARN(get_logger(),
                        "前進を打ち切り (%.1fs経過, dist=%s) → ALIGNへ",
                        elapsed, d.c_str());
            stop_robot();
            transit(State::ALIGN);
            return;
        }

        geometry_msgs::msg::Twist t;
        t.linear.x = approach_v_;
        cmd_vel_pub_->publish(t);
    }

    // 平行旋回: wall_angle が 0 付近になるまで ω を出す
    void do_align()
    {
        const double elapsed = (now() - state_entered_).seconds();

        // 壁検知が死んでいる/平行が出せないまま回り続けるのを防ぐ
        if (elapsed > align_timeout_) {
            RCLCPP_WARN(get_logger(), "平行旋回を打ち切り (%.1fs経過) → CLIMBへ", elapsed);
            stop_robot();
            transit(State::CLIMB);
            return;
        }

        if (!use_wall_detection_ || !wall_angle_fresh()) {
            stop_robot();   // 検知値が無い間は止めて待つ(打ち切りまで)
            return;
        }

        if (std::abs(wall_angle_) < align_tol_) {
            stop_robot();
            RCLCPP_INFO(get_logger(), "平行完了: angle=%.3f", wall_angle_);
            transit(State::CLIMB);
            return;
        }
        // P制御で旋回（並進はゼロ）
        geometry_msgs::msg::Twist t;
        double omega = -align_kp_ * wall_angle_;
        omega = clamp(omega, -align_omega_max_, align_omega_max_);
        t.angular.z = omega;
        cmd_vel_pub_->publish(t);
    }

    // 昇降: /climb/start を1回発行して /climb/done を待つ
    void do_climb()
    {
        if (!climb_.started) {
            std_msgs::msg::Bool b; b.data = true;
            climb_pub_->publish(b);
            climb_.started = true;
            climb_.start_time = now();
            RCLCPP_INFO(get_logger(), "昇降を開始します /climb/start");
            return;
        }
        // タイムアウト保険
        if ((now() - climb_.start_time).seconds() > climb_timeout_) {
            RCLCPP_WARN(get_logger(), "昇降タイムアウト → 次へ強制に遷移します！！！");
            climb_.done = true;
        }
        if (climb_.done) {
            RCLCPP_INFO(get_logger(), "昇降が完了しました。HOME位置に移動します。");
            transit(State::MOVE_TO_HOME);
        }
    }

    // 手動回収: 調停を手動に明け渡し /auto/collect_done を待つ
    void do_manual_collect()
    {
        if (!collect_.handover) {
            publish_arbitration("manual");
            stop_robot();
            collect_.handover = true;
            RCLCPP_INFO(get_logger(), "手動回収へ: コントローラで回収してください");
        }
        if (collect_.done) {
            publish_arbitration("auto");  // 自動に戻す
            RCLCPP_INFO(get_logger(), "回収完了しました → HOME位置へ移動します");
            transit(State::MOVE_TO_SHOOT);
        }
    }

    // 射出: /shooter/fire_request を fire_hold 秒だけ立てる
    void do_fire()
    {
        if (!fire_.started) {
            fire_.start_time = now();
            fire_.started = true;
            RCLCPP_INFO(get_logger(), "射出を要求します /shooter/fire_request");
        }
        std_msgs::msg::Bool b;
        b.data = ((now() - fire_.start_time).seconds() < fire_hold_);
        fire_pub_->publish(b);
        if (!b.data) {
            RCLCPP_INFO(get_logger(), "射出が完了しました → DONE");
            transit(State::DONE);
        }
    }

    // 位置制御: 目標(x,y,yaw)へ /cmd_vel を出す。到達でtrueを返し次へ。
    // arrived_flag=true のとき、到達時に /auto/arrived を publish
    void do_move(double tx, double ty, double tyaw, bool arrived_flag)
    {
        if (!pose_ok_) return;

        const double dx = tx - cur_x_;
        const double dy = ty - cur_y_;
        const double dist = std::hypot(dx, dy);
        double dyaw = normalize(tyaw - cur_yaw_);

        if (dist < pos_tol_ && std::abs(dyaw) < yaw_tol_) {
            stop_robot();
            if (arrived_flag) {
                std_msgs::msg::Bool b; b.data = true;
                arrived_pub_->publish(b);
                RCLCPP_INFO(get_logger(), "定位置に到達。 GUIに通知します。");
                transit(State::MANUAL_COLLECT);
            } else {
                RCLCPP_INFO(get_logger(), "射出位置に到達");
                transit(State::FIRE);
            }
            return;
        }

        // map系の目標方向を機体座標へ（yawで回転）
        geometry_msgs::msg::Twist t;
        const double c = std::cos(cur_yaw_), s = std::sin(cur_yaw_);
        double vx_body =  c*dx + s*dy;
        double vy_body = -s*dx + c*dy;
        double n = std::hypot(vx_body, vy_body);
        if (n > 1e-6) {
            double v = clamp(move_kp_ * dist, 0.0, move_v_max_);
            t.linear.x = v * vx_body / n;
            t.linear.y = v * vy_body / n;
        }
        t.angular.z = clamp(move_kp_yaw_ * dyaw, -move_omega_max_, move_omega_max_);
        cmd_vel_pub_->publish(t);
    }

    // ── ユーティリティ ──────────────────────────
    void stop_robot()
    {
        geometry_msgs::msg::Twist t;  // 全ゼロ
        cmd_vel_pub_->publish(t);
    }

    // 壁検知値の鮮度。wall_detection が落ちた/条件を満たせず publish を止めた場合に
    // 古い値でDETECT_STEPを即抜けしないようにする。
    bool wall_dist_fresh() const
    { return stamp_fresh(wall_dist_stamp_); }

    bool wall_angle_fresh() const
    { return stamp_fresh(wall_angle_stamp_); }

    bool stamp_fresh(const rclcpp::Time& t) const
    {
        if (t.nanoseconds() == 0) return false;   // 未受信
        return (now() - t).seconds() < wall_data_timeout_;
    }

    void transit(State s)
    {
        state_ = s;
        state_entered_ = now();
        // ステート入口のフラグリセット
        climb_.started = false;
        collect_.handover = false;
        fire_.started = false;
        if (s == State::CLIMB) climb_.done = false;
        if (s == State::MANUAL_COLLECT) collect_.done = false;
        RCLCPP_INFO(get_logger(), "→ state: %s", state_name(s));
    }

    void publish_state()
    {
        std_msgs::msg::String m;
        m.data = state_name(state_);
        state_pub_->publish(m);
    }

    void publish_arbitration(const std::string& mode)
    {
        std_msgs::msg::String m;
        m.data = mode;   // "auto" or "manual"
        arb_pub_->publish(m);
    }

    static double clamp(double v, double lo, double hi)
    { return std::max(lo, std::min(hi, v)); }

    static double normalize(double a)
    {
        while (a >  M_PI) a -= 2*M_PI;
        while (a < -M_PI) a += 2*M_PI;
        return a;
    }

    // ── パラメータ ─────────────────────────────
    double step_dist_, approach_v_, approach_timeout_, wall_data_timeout_;
    bool   use_wall_detection_;
    double align_tol_, align_kp_, align_omega_max_, align_timeout_;
    double home_x_, home_y_, home_yaw_;
    double shoot_x_, shoot_y_, shoot_yaw_;
    double pos_tol_, yaw_tol_;
    double move_kp_, move_v_max_, move_kp_yaw_, move_omega_max_;
    double climb_timeout_, fire_hold_, rate_hz_;

    // ── 状態 ───────────────────────────────────
    State state_ = State::IDLE;
    rclcpp::Time state_entered_{0,0,RCL_ROS_TIME};

    double wall_angle_ = 0.0;   rclcpp::Time wall_angle_stamp_{0,0,RCL_ROS_TIME};
    double wall_dist_  = 0.0;   rclcpp::Time wall_dist_stamp_{0,0,RCL_ROS_TIME};
    double cur_x_ = 0, cur_y_ = 0, cur_yaw_ = 0;  bool pose_ok_ = false;

    // 状態ごとのフラグ束(struct のメンバは末尾_なし)
    ClimbState   climb_;
    CollectState collect_;
    FireState    fire_;

    // ── ROS ────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   climb_pub_, collect_pub_, fire_pub_, arrived_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, arb_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_, collect_done_sub_, abort_sub_, climb_done_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr force_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_, dist_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NatsuAutoNode>());
    rclcpp::shutdown();
    return 0;
}
