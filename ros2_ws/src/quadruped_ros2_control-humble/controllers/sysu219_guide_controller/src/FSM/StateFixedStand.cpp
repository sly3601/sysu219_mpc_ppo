//
// Created by biao on 24-9-10.
//

#include "sysu219_guide_controller/FSM/StateFixedStand.h"
#include "sysu219_guide_controller/FSM/StateTrotting.h"

StateFixedStand::StateFixedStand(CtrlInterfaces &ctrl_interfaces, const std::vector<double> &target_pos,
                                 const double kp,
                                 const double kd,
                                 const bool hybrid_policy_slots)
    : BaseFixedStand(ctrl_interfaces, target_pos, kp, kd), hybrid_policy_slots_(hybrid_policy_slots) {
}

std::array<double, 12> StateFixedStand::targetPosition() const {
    std::array<double, 12> result{};
    for (int i = 0; i < 12; ++i) result[i] = target_pos_[i] + offset_pos_[i];
    return result;
}

void StateFixedStand::enter() {
    if (previous_state_name != FSMStateName::COMIDENTIFICATION) {
        BaseFixedStand::enter();
        return;
    }
    // COM has already returned to this state's exact target and confirmed rest.
    // Preserve command/gains: resetting q_cmd to measured q would remove the load-bearing PD torque.
    for (int i = 0; i < 12; ++i) {
        cmd_pos_[i] = ctrl_interfaces_.joint_position_command_interface_[i].get().get_value();
        start_pos_[i] = cmd_pos_[i] - offset_pos_[i];
    }
    percent_ = 20.0; phase = 1.0;
    ctrl_interfaces_.control_inputs_.command = 0;
    fixedstand_active_ = true;
}

FSMStateName StateFixedStand::checkChange() {
    const auto command = ctrl_interfaces_.control_inputs_.command;
    requested_policy_key_ = 0;
    // 失能优先：1 键不受姿态过渡锁定期限制，随时可切
    if (command == 1) {
        return FSMStateName::PASSIVE;
    }
    const bool policy_key = hybrid_policy_slots_ && StateTrotting::isHybridPolicyKey(command);
    if (percent_ < 1.5) {
        if (policy_key && ctrl_interfaces_.control_inputs_.command == command) {
            ctrl_interfaces_.control_inputs_.command = 0;
        }
        return FSMStateName::FIXEDSTAND;
    }
    if (policy_key) {
        requested_policy_key_ = command;
        return FSMStateName::TROTTING;
    }
    switch (command) {
        case 2:
            return FSMStateName::FIXEDDOWN;
        case 8:
            // 8 键：先降到半趴，等半趴过渡完成后再自动继续到全趴
            // （见 StateFixedDown::checkChange）
            ctrl_interfaces_.go_prone_after_down = true;
            return FSMStateName::FIXEDDOWN;
        default:
            return FSMStateName::FIXEDSTAND;
    }
}
