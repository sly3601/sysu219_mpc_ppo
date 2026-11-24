//
// Created by biao on 24-9-10.
//

#ifndef STATEFIXEDSTAND_H
#define STATEFIXEDSTAND_H

#include <controller_common/FSM/BaseFixedStand.h>
#include <array>

class StateFixedStand final : public BaseFixedStand {
public:
    explicit StateFixedStand(CtrlInterfaces &ctrl_interfaces,
                             const std::vector<double> &target_pos,
                             double kp,
                             double kd,
                             bool hybrid_policy_slots = false);

    void enter() override;
    std::array<double, 12> targetPosition() const;
    FSMStateName checkChange() override;
    int requestedPolicyKey() const { return requested_policy_key_; }

private:
    const bool hybrid_policy_slots_;
    int requested_policy_key_ = 0;
};


#endif //STATEFIXEDSTAND_H
