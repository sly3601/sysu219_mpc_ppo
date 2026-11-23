#pragma once

#include <Eigen/Core>
#include <array>
#include <cmath>
#include <cstddef>

// Body-heading reference built solely from command and the actor clock.
// Leg order is FR, FL, RR, RL. Position and velocity refer to the foot-link centers.
class LocalGaitReference {
public:
    void reset() {
        phase_ = kHalfCycle;
        filtered_command_.setZero();
    }

    std::array<float, 26> step(const Eigen::Vector3d& command_body) {
        const double angle = kTau * phase_ / kCycleTicks;
        phase_ = (phase_ + kActorTicks) % kCycleTicks;
        std::array<int, 4> next_contact{};
        std::array<double, 4> next_phase{};
        for (std::size_t leg = 0; leg < kFeet.size(); ++leg) {
            const int leg_tick = (phase_ + kCycleTicks - kFeet[leg].bias) % kCycleTicks;
            next_contact[leg] = leg_tick < kStanceTicks;
            next_phase[leg] = next_contact[leg] ? static_cast<double>(leg_tick) / kStanceTicks
                                               : static_cast<double>(leg_tick - kStanceTicks) / kSwingTicks;
        }
        return plan(command_body, angle, next_contact, next_phase, kActorPeriod, kStance, kSwing);
    }

    std::array<float, 26> stepSynchronized(const Eigen::Vector3d& command_body, double current_cycle,
                                           const std::array<int, 4>& next_contact,
                                           const std::array<double, 4>& next_phase, double sample_interval,
                                           double stance_time, double swing_time) {
        return plan(command_body, kTau * current_cycle, next_contact, next_phase, sample_interval,
                    stance_time, swing_time);
    }

private:
    std::array<float, 26> plan(const Eigen::Vector3d& command_body, double angle,
                               const std::array<int, 4>& next_contact, const std::array<double, 4>& next_phase,
                               double sample_interval, double stance_time, double swing_time) {
        const double alpha = -std::expm1(-sample_interval / kCommandTimeConstant);
        filtered_command_ += alpha * (command_body - filtered_command_);

        std::array<float, 26> reference{};
        reference[0] = static_cast<float>(std::sin(angle));
        reference[1] = static_cast<float>(std::cos(angle));
        for (std::size_t leg = 0; leg < kFeet.size(); ++leg) {
            const FootCenter& foot = kFeet[leg];
            const double vx = filtered_command_.x() - filtered_command_.z() * foot.y;
            const double vy = filtered_command_.y() + filtered_command_.z() * foot.x;
            const double u = next_phase[leg];
            double offset, speed;
            double z = kNominalZ, vz = 0.0;
            if (next_contact[leg]) {
                offset = stance_time * (0.5 - u);
                speed = -1.0;
            } else {
                const double u2 = u * u;
                // Cubic Hermite swing joins the stance velocity at both endpoints.
                offset = stance_time * (u2 * (3.0 - 2.0 * u) - 0.5)
                       - swing_time * (u - 3.0 * u2 + 2.0 * u2 * u);
                speed = stance_time / swing_time * 6.0 * u * (1.0 - u) - (1.0 - 6.0 * u + 6.0 * u2);
                z += 0.5 * kHeight * (1.0 - std::cos(kTau * u));
                vz = 0.5 * kTau * kHeight / swing_time * std::sin(kTau * u);
            }
            const std::size_t index = 3 * leg;
            reference[2 + index] = static_cast<float>(foot.x + vx * offset);
            reference[3 + index] = static_cast<float>(foot.y + vy * offset);
            reference[4 + index] = static_cast<float>(z);
            reference[14 + index] = static_cast<float>(vx * speed);
            reference[15 + index] = static_cast<float>(vy * speed);
            reference[16 + index] = static_cast<float>(vz);
        }
        return reference;
    }

    static constexpr double kTau = 6.2831853071795864769;
    static constexpr double kActorPeriod = 0.020;
    static constexpr double kCommandTimeConstant = 0.18;
    static constexpr double kHeight = 0.07;
    static constexpr double kStance = 0.650;
    static constexpr double kSwing = 0.325;
    // Integer phase uses 0.5 ms units so training and deployment agree at phase boundaries.
    static constexpr int kCycleTicks = 1950, kHalfCycle = kCycleTicks / 2;
    static constexpr int kStanceTicks = 1300, kSwingTicks = kCycleTicks - kStanceTicks;
    static constexpr int kActorTicks = 40;

    struct FootCenter { double x, y; int bias; };
    // Leveled FIXEDSTAND geometry with the source planner's 1 cm forward offset.
    static constexpr double kFrontX = 0.2825648552583156;
    static constexpr double kRearX = -0.4652444922328959;
    static constexpr double kLateral = 0.2183;
    static constexpr double kNominalZ = -0.4173655010846403;
    static constexpr std::array<FootCenter, 4> kFeet{{
        {kFrontX, -kLateral, 0}, {kFrontX, kLateral, kHalfCycle},
        {kRearX, -kLateral, kHalfCycle}, {kRearX, kLateral, 0},
    }};

    int phase_ = kHalfCycle;
    Eigen::Vector3d filtered_command_ = Eigen::Vector3d::Zero();
};
