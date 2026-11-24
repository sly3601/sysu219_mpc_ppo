#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <vector>

#include <control_input_msgs/msg/inputs.hpp>
#include <realtime_tools/realtime_buffer.hpp>
#include <realtime_tools/realtime_publisher.hpp>
#include <std_msgs/msg/empty.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "controller_common/FSM/FSMState.h"
#include "sysu219_guide_controller/control/ContactCoordinator.hpp"
#include "sysu219_guide_controller/control/ConvexMpcSolver.h"
#include "sysu219_guide_controller/control/FlatPpoTracker.h"
#include "sysu219_guide_controller/gait/GaitGenerator.h"
#include "sysu219_guide_controller/debug/foot_marker_publisher.hpp"

class StateTrotting final : public FSMState {
public:
    StateTrotting(CtrlInterfaces& interfaces, CtrlComponent& components, bool pure_gait,
                  bool smooth_gait = false, bool local_gait = false, bool hybrid_gait = false);
    void enter() override;
    void updateSynchronizedGaitClock(const rclcpp::Time& time, const rclcpp::Duration& period);
    void run(const rclcpp::Time& time, const rclcpp::Duration& period) override;
    void exit() override;
    FSMStateName checkChange() override;

    bool faulted() const { return fault_; }
    bool ready() const { return active_tracker_ && active_tracker_->ready(); }
    bool primaryPolicyReady() const { return tracker_.ready(); }
    bool usesEqualHybridPhases() const { return tracker_.usesEqualHybridPhases(); }
    static bool isHybridPolicyKey(int key);
    bool selectHybridPolicy(int key);
    bool jointFeedbackValid() const;
    double getFilteredYawRateGlobal() const { return (rotation_ * gyro_)(2); }

    // Retain the closed-loop gait generator branch used by the original TROTTING controller.
    static constexpr int troting_kalman = 2;
    enum class ForceSolverMode { QP, MPC };
    static constexpr ForceSolverMode force_solver_mode_ = ForceSolverMode::MPC;

private:
    struct MotionInput {
        double lx = 0.0, ly = 0.0, rx = 0.0;
        std::chrono::steady_clock::time_point received_at{};
    };
    using Reference = std_msgs::msg::Float64MultiArray;
    Vec3 requestedVelocityBody() const;
    void updateMotion(double dt);
    bool updatePlan(double now, const Vec34& measured_feet);
    void publishTrottingActive();
    void publishReference(double now, const Vec34& feet_body);
    void fail(const char* reason);

    CtrlComponent& components_;
    const bool pure_gait_;
    const bool smooth_gait_;
    const bool local_gait_;
    const bool hybrid_gait_;
    std::unique_ptr<GaitGenerator> gait_;
    FlatPpoTracker tracker_;
    static constexpr std::array<int, 6> hybrid_policy_keys_{3, 4, 5, 6, 7, 9};
    std::array<std::unique_ptr<FlatPpoTracker>, hybrid_policy_keys_.size()> hybrid_trackers_;
    FlatPpoTracker* active_tracker_;
    int selected_key_ = 0;
    std::unique_ptr<quadruped_controller::FootMarkerPublisher> markers_;
    unsigned marker_tick_ = 0;
    sysu219_guide_controller::ContactCoordinator coordinator_;
    std::shared_ptr<realtime_tools::RealtimeBuffer<MotionInput>> motion_ =
        std::make_shared<realtime_tools::RealtimeBuffer<MotionInput>>();
    rclcpp::Subscription<control_input_msgs::msg::Inputs>::SharedPtr motion_subscription_;
    std::unique_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Empty>> trotting_active_;
    rclcpp::Publisher<Reference>::SharedPtr reference_pub_;
    std::unique_ptr<realtime_tools::RealtimePublisher<Reference>> realtime_pub_;
    std::chrono::steady_clock::time_point entry_at_;
    const std::vector<double> stand_;
    bool first_ = true, fault_ = false, feedback_contacts_ = false;
    Vec3 position_, velocity_, position_goal_, velocity_goal_, command_;
    double yaw_goal_ = 0.0, yaw_rate_goal_ = 0.0;
    double entry_height_ = 0.0, initial_height_ = 0.0, height_elapsed_ = 0.0;
    double last_plan_s_ = -1.0, last_tick_s_ = -1.0;
    std::int64_t gait_start_ns_ = 0, gait_elapsed_ns_ = 0;
    RotMat rotation_;
    Vec3 gyro_, raw1_, raw2_, filtered1_, filtered2_;
    VecInt4 contact_, last_plan_contact_;
    Vec34 feet_goal_, feet_velocity_, feet_hold_;
    std::vector<Vec4> max_load_, min_load_;
};
