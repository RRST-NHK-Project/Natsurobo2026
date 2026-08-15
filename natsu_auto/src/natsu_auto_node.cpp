/*
 * natsu_auto_node.cpp
 *
 * 夏ロボ2026 自律シーケンスFSM(有限オートマトン遷移図)
 *
 * 簡素化版: 昇降(CLIMB)と平行合わせ(ALIGN)、および実行ノード natsu_climb_seq_node
 * への phase_done 受け渡しを廃止。DETECT_STEP は司令塔自身が /wall_detection を
 * 直読みして「検知した場所へ向かう(前進＋壁偏角で操舵)」制御を行い、段まで詰めたら
 * そのまま手動回収へ移る。
 *
 *   IDLE ─(/auto/start)→ DETECT_STEP ─(到達)→ MANUAL_COLLECT ─(回収完了)→ FIRE → DONE
 *
 * subscribe:
 *   /auto/start            [std_msgs/Bool]      シーケンス開始(GUI)
 *   /auto/collect_done     [std_msgs/Bool]      手動回収完了(GUI)
 *   /auto/abort            [std_msgs/Bool]      緊急中断(GUI/PS4)
 *   /auto/force_state      [std_msgs/String]    任意状態へ強制遷移(GUI, 動作完了を待たない)
 *   /wall_detection/angle    [std_msgs/Float64]   壁偏角[rad]
 *   /wall_detection/distance [std_msgs/Float64]   壁距離[m]
 *
 * publish:
 *   /cmd_vel              [geometry_msgs/Twist]  走行指令(→zakiomni)
 *   /collect/start        [std_msgs/Bool]        回収開始(手動運用時は未使用可)
 *   /shooter/fire_request [std_msgs/Bool]        射出要求
 *   /auto/state            [std_msgs/String]      現在状態(GUI表示)
 *   /auto/arrived          [std_msgs/Bool]        手動回収の準備完了(GUI「回収してください」通知)
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

// ──────────────────────────────────────────────
enum class State {
    IDLE,
    DETECT_STEP,
    MANUAL_COLLECT,
    FIRE,
    DONE,
    ABORTED
};

static const char* state_name(State s)
{
    switch (s) {
        case State::IDLE:           return "IDLE";
        case State::DETECT_STEP:    return "DETECT_STEP";
        case State::MANUAL_COLLECT: return "MANUAL_COLLECT";
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
    if (n == "MANUAL_COLLECT") return State::MANUAL_COLLECT;
    if (n == "FIRE")           return State::FIRE;
    if (n == "DONE")           return State::DONE;
    if (n == "ABORTED")        return State::ABORTED;
    return std::nullopt;
}

// --- 状態ごとのフラグ束 (Google C++ Style Guide 準拠) ---
// データを束ねるだけ(不変条件なし)なので struct を使う。
// struct のメンバは末尾アンダースコアを付けない(class のデータメンバとは異なる)。

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
        // 段差検知: 壁距離がこれ以下なら石倉前(到達)とみなす [m]
        declare_parameter<double>("step_detect_distance", 0.30);
        // 石倉に向かって詰めるときの前進速度 [m/s]
        declare_parameter<double>("approach_v", 0.15);
        // 前進の打ち切り [s]。壁検知が出ない/閾値に届かない場合の走行距離上限を決める
        declare_parameter<double>("approach_timeout", 8.0);
        // 壁検知値をこの秒数より古ければ「無効」とみなす [s]
        declare_parameter<double>("wall_data_timeout", 0.5);
        // false にすると壁検知を待たず、前進のみ＋approach_timeout だけで次へ進む
        declare_parameter<bool>("use_wall_detection", true);

        // 接近中の操舵: 壁偏角に比例したyawで正面を向けながら詰める
        declare_parameter<double>("approach_kp", 1.5);        // 旋回Pゲイン
        declare_parameter<double>("approach_omega_max", 1.0); // 最大角速度 [rad/s]
        declare_parameter<double>("approach_angle_sign", 1.0);// 偏角の符号(実機で±調整)

        // 射出パルスの長さ [s]
        declare_parameter<double>("fire_hold", 1.0);

        // 制御周期 [Hz]
        declare_parameter<double>("rate_hz", 20.0);

        load_params();

        // ── Publisher ───────────────────────────────
        cmd_vel_pub_   = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
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
        approach_kp_       = get_parameter("approach_kp").as_double();
        approach_omega_max_ = get_parameter("approach_omega_max").as_double();
        approach_angle_sign_ = get_parameter("approach_angle_sign").as_double();
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

    // ── メインループ ────────────────────────────
    void loop()
    {
        switch (state_) {
            case State::IDLE:           break;  // /auto/start 待ち
            case State::DETECT_STEP:    do_detect_step();   break;
            case State::MANUAL_COLLECT: do_manual_collect(); break;
            case State::FIRE:           do_fire();          break;
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
        return s == State::DETECT_STEP || s == State::FIRE;
    }

    // ── 各ステート処理 ──────────────────────────

    // DETECT_STEP: 検知した場所(石倉/段)へ向かう。
    //   ・/wall_detection/distance で前進し、step_dist_ 以下まで詰めたら到達=手動回収へ
    //   ・/wall_detection/angle に比例したyawで正面を向けながら接近(前進＋操舵)
    //   ・検知が古い/来ない時は安全のため停止し、approach_timeout で打ち切って次へ
    void do_detect_step()
    {
        const double elapsed = (now() - state_entered_).seconds();

        // 打ち切り保険: 検知の有無に関わらず規定時間で必ず止めて次へ進む
        if (elapsed >= approach_timeout_) {
            stop_robot();
            RCLCPP_WARN(get_logger(), "approach_timeout(%.1fs) → 手動回収へ", approach_timeout_);
            transit(State::MANUAL_COLLECT);
            return;
        }

        // 壁検知を使わない運用: まっすぐ前進のみ(打ち切りは上のtimeout)
        if (!use_wall_detection_) {
            geometry_msgs::msg::Twist t;
            t.linear.x = approach_v_;
            cmd_vel_pub_->publish(t);
            return;
        }

        // 検知値が古い/未受信: 古い値で誤って到達判定しないよう停止して待つ
        if (!wall_dist_fresh()) {
            stop_robot();
            return;
        }

        // 到達判定: 段まで step_dist_ 以下に詰めたら完了
        if (wall_dist_ <= step_dist_) {
            stop_robot();
            RCLCPP_INFO(get_logger(), "検知場所に到達 (dist=%.2fm) → 手動回収へ", wall_dist_);
            transit(State::MANUAL_COLLECT);
            return;
        }

        // 前進しつつ、壁偏角に比例したyawで正面を向ける(角度が新しい時のみ操舵)
        geometry_msgs::msg::Twist t;
        t.linear.x = approach_v_;
        if (wall_angle_fresh()) {
            const double omega = approach_angle_sign_ * approach_kp_ * wall_angle_;
            t.angular.z = clamp(omega, -approach_omega_max_, approach_omega_max_);
        }
        cmd_vel_pub_->publish(t);
    }

    // 手動回収: 調停を手動に明け渡し /auto/collect_done を待つ。
    // 到達した位置のまま回収するので移動は挟まず、完了したら直接 FIRE へ。
    void do_manual_collect()
    {
        if (!collect_.handover) {
            publish_arbitration("manual");
            stop_robot();
            // 到達位置＝回収位置。GUIへ「回収してください」を通知
            std_msgs::msg::Bool b; b.data = true;
            arrived_pub_->publish(b);
            collect_.handover = true;
            RCLCPP_INFO(get_logger(), "手動回収へ: コントローラで回収してください");
        }
        if (collect_.done) {
            publish_arbitration("auto");  // 自動に戻す
            RCLCPP_INFO(get_logger(), "回収完了しました → 射出します");
            transit(State::FIRE);
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
        collect_.handover = false;
        fire_.started = false;
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

    // ── パラメータ ─────────────────────────────
    double step_dist_, approach_v_, approach_timeout_, wall_data_timeout_;
    bool   use_wall_detection_;
    double approach_kp_, approach_omega_max_, approach_angle_sign_;
    double fire_hold_, rate_hz_;

    // ── 状態 ───────────────────────────────────
    State state_ = State::IDLE;
    rclcpp::Time state_entered_{0,0,RCL_ROS_TIME};

    double wall_angle_ = 0.0;   rclcpp::Time wall_angle_stamp_{0,0,RCL_ROS_TIME};
    double wall_dist_  = 0.0;   rclcpp::Time wall_dist_stamp_{0,0,RCL_ROS_TIME};

    // 状態ごとのフラグ束(struct のメンバは末尾_なし)
    CollectState collect_;
    FireState    fire_;

    // ── ROS ────────────────────────────────────
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr   collect_pub_, fire_pub_, arrived_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_pub_, arb_pub_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr start_sub_, collect_done_sub_, abort_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr force_state_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr angle_sub_, dist_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NatsuAutoNode>());
    rclcpp::shutdown();
    return 0;
}
