#include "sysu219_guide_controller/FSM/StateTrotting.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <rclcpp/create_publisher.hpp>

#include "sysu219_guide_controller/common/mathTools.h"
#include "sysu219_guide_controller/control/CtrlComponent.h"

namespace {
constexpr double kSolvePeriod = 0.008;       // original 125 Hz MPC
constexpr double kPredictionStep = 0.020;    // original 20 ms MPC grid
constexpr double kMaxPlanAge = 0.020;       // one policy interval
constexpr double kGaitHeight = 0.07;
}

StateTrotting::StateTrotting(CtrlInterfaces& interfaces, CtrlComponent& components, bool pure_gait,
                             bool smooth_gait, bool local_gait, bool hybrid_gait)
    : FSMState(FSMStateName::TROTTING, "ppo_trotting", interfaces),
      components_(components), pure_gait_(pure_gait), smooth_gait_(smooth_gait), local_gait_(local_gait),
      hybrid_gait_(hybrid_gait), tracker_(interfaces, hybrid_gait_ ? FlatPpoTracker::Mode::HybridGait :
          local_gait_ ? FlatPpoTracker::Mode::LocalGait :
          smooth_gait_ ? FlatPpoTracker::Mode::SmoothGait :
          pure_gait_ ? FlatPpoTracker::Mode::PureGait :
          (interfaces.node->get_parameter("mpc_ppo_foot_tracking").as_bool() ?
               FlatPpoTracker::Mode::FootTrackingMpc : FlatPpoTracker::Mode::FlatMpc)),
      stand_(interfaces.node->get_parameter("stand_pos").as_double_array()) {
    motion_subscription_ = interfaces.node->create_subscription<control_input_msgs::msg::Inputs>(
        "/trotting_motion_input", rclcpp::QoS(1),
        [buffer = motion_](const control_input_msgs::msg::Inputs& msg) {
            buffer->writeFromNonRT({msg.ly, msg.rx, std::chrono::steady_clock::now()});
        });
    auto status = rclcpp::create_publisher<std_msgs::msg::Empty>(
        interfaces.node, "/trotting_active", rclcpp::QoS(1).best_effort());
    trotting_active_ = std::make_unique<realtime_tools::RealtimePublisher<std_msgs::msg::Empty>>(status);
    if (pure_gait_) return;
    gait_ = std::make_unique<GaitGenerator>(components, this);
    markers_ = std::make_unique<quadruped_controller::FootMarkerPublisher>(interfaces.node);
    max_load_.reserve(128);
    min_load_.reserve(128);
    if (interfaces.node->get_parameter("use_sim_time").as_bool()) {
        reference_pub_ = rclcpp::create_publisher<Reference>(
            interfaces.node, "/mpc_ppo/reference", rclcpp::QoS(10));
        realtime_pub_ = std::make_unique<realtime_tools::RealtimePublisher<Reference>>(reference_pub_);
        realtime_pub_->msg_.data.resize(20);
    }
}

bool StateTrotting::jointFeedbackValid() const {
    if (ctrl_interfaces_.joint_position_state_interface_.size() != 12 ||
        ctrl_interfaces_.joint_velocity_state_interface_.size() != 12) return false;
    for (int i = 0; i < 12; ++i) {
        if (!std::isfinite(ctrl_interfaces_.joint_position_state_interface_[i].get().get_value()) ||
            !std::isfinite(ctrl_interfaces_.joint_velocity_state_interface_[i].get().get_value())) return false;
    }
    return true;
}

void StateTrotting::updateHybridGaitClock(const rclcpp::Time& time, const rclcpp::Duration& period) {
    // Called before InEKF takes its joint/IMU snapshot, including the first PPO tick.
    if (first_) hybrid_gait_start_ns_ = time.nanoseconds();
    hybrid_gait_elapsed_ns_ = std::max<std::int64_t>(0, time.nanoseconds() - hybrid_gait_start_ns_);
    components_.wave_generator_->updateSynchronizedTrot(hybrid_gait_elapsed_ns_, period.seconds());
}

void StateTrotting::enter() {
    first_ = true;
    fault_ = false;
    last_plan_s_ = last_tick_s_ = -1.0;
    entry_at_ = std::chrono::steady_clock::now();
    last_plan_contact_.setZero();
    contact_.setOnes();
    tracker_.reset();
    ctrl_interfaces_.control_inputs_.command = 0;
    if (!ready() || !jointFeedbackValid()) {
        fail("actor or joint feedback unavailable at entry");
        return;
    }
    if (local_gait_) return;
    components_.wave_generator_->status_ = WaveStatus::STANCE_ALL;
    if (pure_gait_) return;
    if (!components_.convex_mpc_) {
        fail("MPC policy selected without configuring its solver");
        return;
    }
    components_.convex_mpc_->reset();
    auto& estimator = *components_.estimator_;
    position_goal_ = estimator.getPosition();
    rotation_ = estimator.getRotation();
    yaw_goal_ = estimator.getYaw();
    yaw_rate_goal_ = 0.0;
    velocity_goal_.setZero();
    height_elapsed_ = 0.0;
    entry_height_ = position_goal_.z();
    feet_hold_ = estimator.getFeetPos();
    raw1_ = raw2_ = filtered1_ = filtered2_ = gyro_ = estimator.getGyro();
    if (!position_goal_.allFinite() || !rotation_.allFinite() || !std::isfinite(yaw_goal_) ||
        !feet_hold_.allFinite() || !gyro_.allFinite()) {
        fail("invalid estimator state at entry");
        return;
    }
    feedback_contacts_ = components_.contact_feedback_enabled;
    coordinator_.reset(feet_hold_);
    gait_->setNominalStand(stand_, RotMat::Identity());
    gait_->restart();
}

Vec3 StateTrotting::requestedVelocityBody() const {
    Vec3 command = Vec3::Zero();
    const auto motion = *motion_->readFromRT();
    if (motion.received_at >= entry_at_ &&
        std::chrono::steady_clock::now() - motion.received_at <= std::chrono::milliseconds(250)) {
        if (std::isfinite(motion.ly)) command.x() = 0.3 * std::clamp(motion.ly, -1.0, 1.0);
        if (std::isfinite(motion.rx)) command.z() = -0.3 * std::clamp(motion.rx, -1.0, 1.0);
    }
    return command;
}

void StateTrotting::updateMotion(double dt) {
    Vec3 command = Vec3::Zero();
    if (!(feedback_contacts_ && coordinator_.recovering())) command = requestedVelocityBody();
    yaw_rate_goal_ = command.z();
    velocity_goal_ = rotz(components_.estimator_->getYaw()) * Vec3(command.x(), 0., 0.);
    position_goal_.head<2>() += dt * velocity_goal_.head<2>();
    yaw_goal_ += dt * yaw_rate_goal_;
    const double duration = components_.wave_generator_->getGaitPeriod();
    if (first_) initial_height_ = position_.z();
    const double u = std::min(1.0, height_elapsed_ / duration);
    const double delta = entry_height_ - initial_height_;
    position_goal_.z() = initial_height_ + delta * u*u*u * (10.0 + u*(-15.0 + 6.0*u));
    velocity_goal_.z() = delta * 30.0 * u*u * (1.0-u)*(1.0-u) / duration;
    height_elapsed_ = std::min(duration, height_elapsed_ + dt);
}

bool StateTrotting::updatePlan(double now, const Vec34& measured_feet) {
    auto& mpc = *components_.convex_mpc_;
    const auto& wave = *components_.wave_generator_;
    const double age = now - last_plan_s_;
    if (last_plan_s_ >= 0.0 && (age < 0.0 || age > kMaxPlanAge + 1e-9)) return false;
    const bool loading = feedback_contacts_ &&
        ((coordinator_.load().array() < 1.0) && (contact_.array() == 1)).any();
    if (!first_ && age + 1e-9 < kSolvePeriod && contact_ == last_plan_contact_ && !loading) return true;

    for (int leg = 0; leg < 4; ++leg)
        if (contact_(leg)) feet_hold_.col(leg) = measured_feet.col(leg);
    const double period = wave.getGaitPeriod();
    auto table = wave.getMpcContactTable(mpc.predictionSteps(kPredictionStep, period), kPredictionStep);
    Vec4 caps = Vec4::Constant(mpc.normalForceMax());
    if (feedback_contacts_) {
        coordinator_.preview(table, max_load_, min_load_, kPredictionStep, mpc.normalForceMax());
        caps = coordinator_.load() * mpc.normalForceMax();
    }
    mpc.solveFromDogWrench(position_goal_, feet_hold_, gait_->getEndFeetPos(), contact_, table,
        kPredictionStep, period, position_, velocity_, rotation_, rotation_ * gyro_,
        rotz(yaw_goal_), velocity_goal_, caps,
        feedback_contacts_ ? &max_load_ : nullptr, feedback_contacts_ ? &min_load_ : nullptr);
    if (!mpc.getPreviewBodyCommand(command_)) return false;
    // Matches the Flat PPO training velocity-command distribution (m/s, m/s, rad/s).
    // Keep one shared bounded value for actor inference and trajectory replay.
    command_ = FlatPpoTracker::boundCommand(command_);
    last_plan_s_ = now;
    last_plan_contact_ = contact_;
    return true;
}

void StateTrotting::run(const rclcpp::Time& time, const rclcpp::Duration& period) {
    if (fault_) return;
    try {
        const double dt = period.seconds();
        const double now = time.seconds();
        if (!std::isfinite(dt) || dt <= 0.0 || dt > kMaxPlanAge + 1e-9 ||
            !std::isfinite(now) || (last_tick_s_ >= 0.0 && now <= last_tick_s_)) {
            fail("control clock stalled or exceeded one actor period");
            return;
        }
        last_tick_s_ = now;
        if (local_gait_) {
            if (!tracker_.stepLocal(dt, requestedVelocityBody())) {
                fail("local gait actor rejected feedback or action");
                return;
            }
            if (tracker_.didInfer()) publishTrottingActive();
            first_ = false;
            return;
        }
        auto& estimator = *components_.estimator_;
        auto& wave = *components_.wave_generator_;
        if (hybrid_gait_) {
            if (!tracker_.stepHybrid(dt, requestedVelocityBody(), estimator.getPosition(), estimator.getYaw(),
                                     hybrid_gait_elapsed_ns_, wave)) {
                fail("hybrid gait actor rejected synchronized phase, odometry or action");
                return;
            }
            if (tracker_.didInfer()) publishTrottingActive();
            first_ = false;
            return;
        }
        if (first_) wave.startTrotFromStand();
        if (pure_gait_) {
            const Vec3 velocity_world = estimator.getVelocity();
            const Vec3 measured = estimator.getRotation().transpose() * velocity_world;
            const Vec3 command = requestedVelocityBody();
            bool accepted;
            if (smooth_gait_) {
                const SmoothGaitReference::Measurement reference{
                    estimator.getPosition(), velocity_world, estimator.getYaw(), estimator.getDYaw(),
                    estimator.getFeetPos()
                };
                accepted = tracker_.stepSmooth(dt, measured, command, reference);
            } else {
                const double cycle = wave.getPhaseControlTime() / wave.getGaitPeriod();
                accepted = tracker_.step(dt, measured, command, Vec34::Zero(), VecInt4::Zero(), cycle);
            }
            if (!accepted) {
                fail("pure gait actor rejected feedback or action");
                return;
            }
            if (tracker_.didInfer()) publishTrottingActive();
            first_ = false;
            return;
        }
        position_ = estimator.getPosition();
        velocity_ = estimator.getVelocity();
        rotation_ = estimator.getRotation();
        const Vec3 raw = estimator.getGyro();
        const Vec34 measured_feet = estimator.getFeetPos();
        if (!position_.allFinite() || !velocity_.allFinite() || !rotation_.allFinite() ||
            !raw.allFinite() || !measured_feet.allFinite() || !jointFeedbackValid()) {
            fail("invalid robot feedback");
            return;
        }
        // Original MPC gyro filter uses the configured control rate, independent of scheduler jitter.
        const double k = std::tan(M_PI * 20.0 / ctrl_interfaces_.frequency_);
        const double a = 1.0 + std::sqrt(2.0)*k + k*k;
        gyro_ = (k*k * (raw + 2.0*raw1_ + raw2_) - 2.0*(k*k-1.0)*filtered1_
               - (1.0-std::sqrt(2.0)*k+k*k)*filtered2_) / a;
        raw2_ = raw1_; raw1_ = raw; filtered2_ = filtered1_; filtered1_ = gyro_;

        if (feedback_contacts_ && !coordinator_.update(dt, wave, components_.contact_observation,
                components_.contact_observation_ready, measured_feet, components_.convex_mpc_->normalForceMax(),
                components_.convex_mpc_->nominalWeight(), last_plan_s_ >= 0.0)) {
            fail(coordinator_.reason());
            return;
        }
        contact_ = feedback_contacts_ ? coordinator_.support() : wave.contact_;
        updateMotion(dt);
        gait_->setGait(velocity_goal_.head<2>(), yaw_rate_goal_, kGaitHeight);
        gait_->generate(feet_goal_, feet_velocity_, feedback_contacts_ ? &coordinator_.motionContact() : nullptr,
                       feedback_contacts_ ? &coordinator_.phase() : nullptr);
        if (feedback_contacts_) coordinator_.filterTargets(feet_goal_, feet_velocity_);
        if (coordinator_.fault() || !feet_goal_.allFinite() || !updatePlan(now, measured_feet)) {
            fail("no fresh, finite contact and MPC preview");
            return;
        }

        const Vec34 feet_body = rotation_.transpose() * (feet_goal_.colwise() - position_);
        const Vec3 velocity_body = rotation_.transpose() * velocity_;
        if (!tracker_.step(dt, velocity_body, command_, feet_body, contact_)) {
            fail("PPO inference or command validation failed");
            return;
        }
        if (tracker_.didInfer()) {
            publishTrottingActive();
            publishReference(now, feet_body);
        }
        if (++marker_tick_ == 10) {
            marker_tick_ = 0;
            std::array<geometry_msgs::msg::Point, 4> points;
            for (int leg = 0; leg < 4; ++leg) {
                points[leg].x = feet_goal_(0, leg);
                points[leg].y = feet_goal_(1, leg);
                points[leg].z = feet_goal_(2, leg);
            }
            markers_->update(points);
            markers_->publish();
        }
        first_ = false;
    } catch (const std::exception& error) {
        fail(error.what());
    }
}

void StateTrotting::publishTrottingActive() {
    if (trotting_active_->trylock()) trotting_active_->unlockAndPublish();
}

void StateTrotting::publishReference(double now, const Vec34& feet_body) {
    if (!reference_pub_ || !reference_pub_->get_subscription_count() || !realtime_pub_->trylock()) return;
    auto& data = realtime_pub_->msg_.data;
    data[0] = now;
    for (int axis = 0; axis < 3; ++axis) data[1 + axis] = command_(axis);
    for (int leg = 0; leg < 4; ++leg) {
        for (int axis = 0; axis < 3; ++axis) data[4 + 3*leg + axis] = feet_body(axis, leg);
        data[16 + leg] = contact_(leg);
    }
    realtime_pub_->unlockAndPublish();
}

void StateTrotting::fail(const char* reason) {
    fault_ = true;
    RCLCPP_ERROR(ctrl_interfaces_.node->get_logger(), "[PPO_TROTTING] %s", reason);
}

void StateTrotting::exit() {
    tracker_.reset();
    if (!local_gait_) components_.wave_generator_->status_ = WaveStatus::STANCE_ALL;
}

FSMStateName StateTrotting::checkChange() {
    if (ctrl_interfaces_.control_inputs_.command == 1 || (fault_ && !jointFeedbackValid()))
        return FSMStateName::PASSIVE;
    if (ctrl_interfaces_.control_inputs_.command == 2 || fault_) return FSMStateName::FIXEDSTAND;
    return FSMStateName::TROTTING;
}
