#pragma once

#include "sysu219_guide_controller/control/SmoothGaitReference.h"

#include <Eigen/Core>
#include <cstdint>
#include <memory>

struct CtrlInterfaces;
class WaveGenerator;

// Construct during controller activation. Body axes: x forward, y left, z up.
// footB columns and contacts use ROS gait leg order FR, FL, RR, RL.
class FlatPpoTracker {
public:
    enum class Mode { FlatMpc, FootTrackingMpc, PureGait, SmoothGait, LocalGait, HybridGait };
    explicit FlatPpoTracker(CtrlInterfaces& interfaces, Mode mode);
    ~FlatPpoTracker();
    static Eigen::Vector3d boundCommand(const Eigen::Vector3d& command);
    bool ready() const;
    bool didInfer() const;  // Last successful step computed a fresh actor output.
    bool usesEqualHybridPhases() const;
    void reset();
    // A false return leaves all motor interfaces untouched. The enclosing FSM
    // must take over on this tick. A reset starts a fresh standing transition.
    bool step(double dt, const Eigen::Vector3d& measuredVelocityBody,
              const Eigen::Vector3d& commandBody, const Eigen::Matrix<double, 3, 4>& footB,
              const Eigen::Matrix<int, 4, 1>& contacts, double gaitCycle = 0.0);
    bool stepSmooth(double dt, const Eigen::Vector3d& measuredVelocityBody,
                    const Eigen::Vector3d& commandBody, const SmoothGaitReference::Measurement& measurement);
    bool stepLocal(double dt, const Eigen::Vector3d& commandBody);
    bool stepHybrid(double dt, const Eigen::Vector3d& commandBody, const Eigen::Vector3d& positionWorld,
                    const Eigen::Vector3d& velocityWorld, double yawWorld,
                    std::int64_t gaitElapsedNs, const WaveGenerator& wave);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
