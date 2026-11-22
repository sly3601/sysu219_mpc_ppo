#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <stdexcept>

// Inference counterpart of ppo_gait.py / SmoothTrotCommand. One call per
// 50 Hz actor update. All input foot positions are foot-link centers, in gait
// order FR, FL, RR, RL. The actor receives the present clock and next targets.
class SmoothGaitReference {
public:
    struct Measurement {
        Eigen::Vector3d position_w;
        Eigen::Vector3d velocity_w;
        double yaw;
        double yaw_rate_w;
        Eigen::Matrix<double, 3, 4> feet_w;
    };

    void reset() { entry_tick_ = 0; }

    std::array<float, 26> step(const Measurement& measured, const Eigen::Vector3d& command_body) {
        if (!measured.position_w.allFinite() || !measured.velocity_w.allFinite()
            || !measured.feet_w.allFinite() || !std::isfinite(measured.yaw)
            || !std::isfinite(measured.yaw_rate_w) || !command_body.allFinite()) {
            throw std::invalid_argument("Nonfinite smooth gait state or velocity command");
        }
        if (entry_tick_ == 0) {
            initialize(measured);
            ++entry_tick_;
        } else if (entry_tick_ == 1) {
            // The first fresh measurement re-anchors the all-stance geometry.
            // Training runs _start_clock and then _plan_next at this tick.
            initialize(measured);
            advance(measured, command_body);
            ++entry_tick_;
        }

        std::array<float, 26> result{};
        result[0] = clock_sin_;
        result[1] = clock_cos_;
        advance(measured, command_body);
        for (std::size_t leg = 0; leg < legs_.size(); ++leg) {
            const Eigen::Vector3d relative = legs_[leg].position_w - measured.position_w;
            const Eigen::Vector3d& velocity = legs_[leg].velocity_w;
            const auto relative_xy = rotate(relative.head<2>(), -measured.yaw);
            const auto velocity_xy = rotate(velocity.head<2>(), -measured.yaw);
            const std::size_t column = 3 * leg;
            result[2 + column] = static_cast<float>(relative_xy.x());
            result[3 + column] = static_cast<float>(relative_xy.y());
            result[4 + column] = static_cast<float>(relative.z());
            result[14 + column] = static_cast<float>(velocity_xy.x());
            result[15 + column] = static_cast<float>(velocity_xy.y());
            result[16 + column] = static_cast<float>(velocity.z());
        }
        if (!std::all_of(result.begin(), result.end(), [](float x) { return std::isfinite(x); })) {
            throw std::runtime_error("Invalid smooth gait reference");
        }
        return result;
    }

private:
    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kActorDt = 0.020;
    static constexpr double kPeriod = 0.975;
    static constexpr double kStance = 0.650;
    static constexpr double kSwing = 0.325;
    static constexpr double kHeight = 0.07;
    static constexpr double kHalfWidth = 0.094 + 0.1243;
    static constexpr std::array<double, 4> kPhaseBias{0.0, 0.5, 0.5, 0.0};

    struct Leg {
        Eigen::Vector3d position_w = Eigen::Vector3d::Zero();
        Eigen::Vector3d velocity_w = Eigen::Vector3d::Zero();
        Eigen::Vector3d anchor_w = Eigen::Vector3d::Zero();
        Eigen::Vector2d landing_w = Eigen::Vector2d::Zero();
        Eigen::Vector2d entry_layout = Eigen::Vector2d::Zero();
        double phase = 0.5;
        double liftoff_z_error = 0.0;
        bool stance = true;
    };

    static Eigen::Vector2d rotate(const Eigen::Vector2d& p, double angle) {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return Eigen::Vector2d(c * p.x() - s * p.y(), s * p.x() + c * p.y());
    }

    void refreshEntryLayout(const Measurement& measured) {
        for (std::size_t pair = 0; pair < legs_.size(); pair += 2) {
            const auto right = rotate((measured.feet_w.col(pair) - measured.position_w).head<2>(), -measured.yaw);
            const auto left = rotate((measured.feet_w.col(pair + 1) - measured.position_w).head<2>(), -measured.yaw);
            const double center_y = 0.5 * (right.y() + left.y());
            legs_[pair].entry_layout = Eigen::Vector2d(right.x(), center_y - kHalfWidth);
            legs_[pair + 1].entry_layout = Eigen::Vector2d(left.x(), center_y + kHalfWidth);
        }
    }

    void initialize(const Measurement& measured) {
        elapsed_ = 0.0;
        clock_sin_ = 0.0f;
        clock_cos_ = -1.0f;
        has_lifted_ = false;
        const double floor_center_z = measured.feet_w.row(2).minCoeff();
        for (std::size_t leg = 0; leg < legs_.size(); ++leg) {
            Leg& state = legs_[leg];
            state = Leg{};
            state.anchor_w = measured.feet_w.col(leg);
            state.anchor_w.z() = floor_center_z;
            state.position_w = state.anchor_w;
            state.landing_w = state.anchor_w.head<2>();
        }
        refreshEntryLayout(measured);
    }

    static Eigen::Vector2d landing(const Leg& state, double phase, const Measurement& measured,
                                   const Eigen::Vector3d& command) {
        const auto commanded_w = rotate(command.head<2>(), measured.yaw);
        const Eigen::Vector2d velocity_error =
            Eigen::Vector2d(0.5, 0.25).cwiseProduct(measured.velocity_w.head<2>() - commanded_w);
        const double remaining = (1.0 - phase) * kSwing;
        Eigen::Vector2d translation = (commanded_w + velocity_error) * remaining
            + commanded_w * (kStance / 2.0) + velocity_error * (kSwing / 2.0 + 0.005);
        translation = translation.cwiseMax(-10.135).cwiseMin(10.135);

        const double commanded_yaw = command.z() * (remaining + kStance / 2.0);
        const double extra_yaw = commanded_yaw * (1.2 * std::abs(commanded_yaw) / (std::abs(commanded_yaw) + 0.1));
        const double predicted_yaw = measured.yaw_rate_w * remaining + command.z() * (kStance / 2.0)
            + (measured.yaw_rate_w - command.z()) * (kSwing / 2.0 - 0.005);
        const double delta_yaw = std::clamp(predicted_yaw, -0.1 + std::min(extra_yaw, 0.0),
                                            0.1 + std::max(extra_yaw, 0.0));

        const Eigen::Vector2d layout = state.entry_layout + Eigen::Vector2d(0.01, 0.0);
        const Eigen::Vector2d predicted = measured.position_w.head<2>() + translation
            + rotate(layout, measured.yaw + delta_yaw);
        const auto foot_offset = rotate(layout, measured.yaw);
        const Eigen::Vector2d tangent(-foot_offset.y(), foot_offset.x());
        const Eigen::Vector2d stride = kPeriod * (commanded_w + command.z() * tangent);
        const double magnitude = stride.norm();
        const Eigen::Vector2d extension = stride * (1.2 * magnitude / (magnitude + 0.04));
        const Eigen::Vector2d lower = Eigen::Vector2d::Constant(-0.04) + extension.cwiseMin(0.0);
        const Eigen::Vector2d upper = Eigen::Vector2d::Constant(0.04) + extension.cwiseMax(0.0);
        return state.anchor_w.head<2>() + (predicted - state.anchor_w.head<2>()).cwiseMax(lower).cwiseMin(upper);
    }

    void advance(const Measurement& measured, const Eigen::Vector3d& command) {
        elapsed_ += kActorDt;
        const double cycle = std::fmod(0.5 + elapsed_ / kPeriod, 1.0);
        const float angle = static_cast<float>(2.0 * kPi * cycle);
        clock_sin_ = std::sin(angle);
        clock_cos_ = std::cos(angle);
        if (!has_lifted_) {
            refreshEntryLayout(measured);
        }

        for (std::size_t leg = 0; leg < legs_.size(); ++leg) {
            Leg& state = legs_[leg];
            double leg_cycle = cycle - kPhaseBias[leg];
            if (leg_cycle < 0.0) {
                leg_cycle += 1.0;
            }
            const bool stance = leg_cycle < kStance / kPeriod;
            // Training stores leg phase as float even though its elapsed clock
            // is double. Keep the halfway landing decision on that same grid.
            const double phase = static_cast<float>(stance ? leg_cycle * kPeriod / kStance
                                                          : (leg_cycle * kPeriod - kStance) / kSwing);

            if (stance) {
                if (!state.stance) {
                    state.anchor_w.head<2>() = state.landing_w;
                }
                state.position_w = state.anchor_w;
                state.velocity_w.setZero();
            } else {
                const bool liftoff = state.stance;
                if (liftoff) {
                    has_lifted_ = true;
                    state.liftoff_z_error = measured.feet_w(2, leg) - state.anchor_w.z();
                }
                if (phase < 0.5) {
                    state.landing_w = landing(state, phase, measured, command);
                }

                const double previous_phase = liftoff ? 0.0 : state.phase;
                const Eigen::Vector2d start = liftoff ? Eigen::Vector2d(measured.feet_w.col(leg).head<2>())
                                                      : Eigen::Vector2d(state.position_w.head<2>());
                const Eigen::Vector2d start_velocity =
                    liftoff ? Eigen::Vector2d(0.0, 0.0) : Eigen::Vector2d(state.velocity_w.head<2>());
                const double remaining = std::max(1.0 - previous_phase, 1.e-7);
                const double duration = remaining * kSwing;
                const double u = std::clamp((phase - previous_phase) / remaining, 0.0, 1.0);
                const double one_minus_u = 1.0 - u;
                const Eigen::Vector2d delta = state.landing_w - start;
                state.position_w.head<2>() = start + u * u * (3.0 - 2.0 * u) * delta
                    + duration * u * one_minus_u * one_minus_u * start_velocity;
                state.velocity_w.head<2>() = (6.0 * u * one_minus_u / duration) * delta
                    + (1.0 - 4.0 * u + 3.0 * u * u) * start_velocity;

                const double theta = 2.0 * kPi * phase;
                const double one_minus_cos = 1.0 - std::cos(theta);
                const double blend = phase - std::sin(theta) / (2.0 * kPi);
                state.position_w.z() = state.anchor_w.z() + 0.5 * kHeight * one_minus_cos
                    + (1.0 - blend) * state.liftoff_z_error;
                state.velocity_w.z() = (kPi * kHeight * std::sin(theta) - one_minus_cos * state.liftoff_z_error) / kSwing;
            }
            state.stance = stance;
            state.phase = phase;
        }
    }

    std::array<Leg, 4> legs_{};
    double elapsed_ = 0.0;
    float clock_sin_ = 0.0f;
    float clock_cos_ = -1.0f;
    int entry_tick_ = 0;
    bool has_lifted_ = false;
};
