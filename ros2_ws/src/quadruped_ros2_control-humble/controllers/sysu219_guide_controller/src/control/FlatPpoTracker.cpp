#include "sysu219_guide_controller/control/FlatPpoTracker.h"

#include "controller_common/CtrlInterfaces.h"
#include "sysu219_guide_controller/control/LocalGaitReference.h"
#include "sysu219_guide_controller/gait/WaveGenerator.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Geometry>
#include <onnxruntime_cxx_api.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
constexpr double kPeriod = 0.02;
constexpr double kControllerPeriod = 0.004;
constexpr std::int64_t kPolicyPeriodNs = 20'000'000, kControllerPeriodNs = 4'000'000;
constexpr double kSecondsPerNanosecond = 1.0e-9;
constexpr double kTargetTimeConstant = 0.010;
constexpr double kHybridOdometryTimeConstant = 1.0;
const float kHybridOdometryAlpha = static_cast<float>(-std::expm1(-kPeriod / kHybridOdometryTimeConstant));
constexpr float kHybridVelocityOnlyAlpha = 0.6726238018f;  // 10 Hz digital cutoff at 50 Hz.
constexpr std::array<const char*, 4> kLegs{"FL", "FR", "RL", "RR"};  // Isaac articulation order
constexpr std::array<const char*, 3> kKinds{"hip", "thigh", "calf"};
constexpr std::array<double, 3> kPose{0., 0.67, -1.30}, kKp{285., 265., 265.};
constexpr std::array<double, 3> kLow{-0.863, -2.1, -2.818}, kHigh{0.863, 4.501, -0.888};
constexpr std::array<double, 3> kCommandLimit{0.5, 0.05, 0.5};
// Intersection of the actor, URDF and downstream leg PD controller limits.
constexpr std::array<double, 3> kGaitLow{-0.638, -0.623, -2.322}, kGaitHigh{0.748, 1.65, -0.888};
constexpr char kGaitContract[] = "3ab3ec864488a0d2fdab999b7ccaba431696e8c27b87a751f1a7de1ad4d0fb45";
constexpr char kSmoothGaitContract[] = "98238c0bf8f22c9f2f9551f4a1c56e0f3881c6c56eb66bc504bbdb0fe08951d3";
constexpr char kLocalGaitContract[] = "f099e8b4e74eaf6f0b0c2df1aeae13329ef02cab8a75904f5a619583f36bb38c";
constexpr char kHybridGaitContract[] = "ef3bfbb7be7b1350eca8b7ef3211e9070947c6635f78384e936877da3fce1825";
constexpr char kDiagonalHybridGaitContract[] = "3320576f27f15e63d83b01b13b0f743e3f9c46b4e626a0e8adff40fc345c6d48";
constexpr char kHybridVelocityGaitId[] = "sysu219.pure_hybrid_gait77.v1";
constexpr char kHybridVelocityGaitContract[] = "9f517ad3d0bf8e566e2d365d17c1a3a455db3c4c644ad8711d7eedbb0403c055";
constexpr char kDiagonalHybridVelocityGaitContract[] = "4e59ccf367ca6e9de22f18127448059f482187b8da137e26ea765398bc2b61bc";
constexpr char kHybridVelocityOnlyId[] = "sysu219.pure_hybrid_gait74.velocity_odom.v1";
constexpr char kHybridVelocityOnlyContract[] = "52c68f5e7fba8c69ff1140794f2bac13e23cdca91c81ff0f112e4dbf4e0e644d";
constexpr char kHybridFilteredOdometryId[] = "sysu219.pure_hybrid_gait75.filtered_odom.v1";
constexpr char kHybridFilteredOdometryContract[] = "40935967948bccd657c3cbdee7e6d9980e0d7d62d90ebadcdf581ac8dafce298";
constexpr char kHybridXYHoldId[] = "sysu219.pure_hybrid_gait75.xy_hold.v1";
constexpr char kHybridXYHoldContract[] = "e232761e259c78810a7a200f6783ba9d718c9f5fd6661cb222c9270e57cbe8aa";
constexpr char kHybridXYHoldObservationTrialContract[] = "c44dc19c28718d283f2919867f77c85bd0fb8635c662d3ab4730513940ce8dca";
constexpr char kKey9Checkpoint[] = "5b5db66f4034be1412bea800a45af0e18703aaf7c778d4b9ecb2481f1a1d0b73";
constexpr char kFlatCheckpoint[] = "5d16c702b3bba579830968f16c85722dbb06d75e7f1b6efd922fef3010363228";
constexpr char kFlatContract[] = "e0f63c48ba5536a61af412bffd122f650b3a18fed27b0f3210f9465d61c8d4ba";
constexpr char kTrackingContract[] = "930d512bfd01635766017ffa1368a462b5c42879b034827256b82d815737dd65";
constexpr std::size_t kProprioceptionWidth = 45;  // All feedback and past actions except base linear velocity.
constexpr std::size_t kLocalGaitWidth = kProprioceptionWidth + 26;
constexpr std::size_t kHybridPositionWidth = kLocalGaitWidth + 3;
constexpr std::size_t kHybridVelocityWidth = kHybridPositionWidth + 3;
constexpr std::size_t kHybridFilteredOdometryWidth = kLocalGaitWidth + 4;

struct PolicySpec {
    const char* model;
    const char* id;
    const char* fingerprint;
    int64_t width;
    bool pure;
    bool filtered_targets;
};

PolicySpec policyFor(FlatPpoTracker::Mode mode) {
    using Mode = FlatPpoTracker::Mode;
    switch (mode) {
        case Mode::FlatMpc:
            return {"mpc_flat_ppo.onnx", "sysu219.flat48.v1", kFlatContract, 48, false, false};
        case Mode::FootTrackingMpc:
            return {"mpc_foot_tracker.onnx", "sysu219.mpc_foot64.v1", kTrackingContract, 64, false, false};
        case Mode::PureGait:
            return {"pure_gait_ppo.onnx", "sysu219.pure_gait50.v2", kGaitContract, 50, true, false};
        case Mode::SmoothGait:
            return {"pure_smooth_gait_ppo.onnx", "sysu219.pure_gait74.v1", kSmoothGaitContract, 74, true, true};
        case Mode::LocalGait:
            return {"pure_local_gait_ppo.onnx", "sysu219.pure_local_gait71.v1", kLocalGaitContract, 71, true, true};
        case Mode::HybridGait:
            return {"pure_hybrid_gait_ppo.onnx", "sysu219.pure_hybrid_gait74.v1",
                    kHybridGaitContract, kHybridPositionWidth, true, true};
    }
    throw std::invalid_argument("Unknown PPO deployment mode");
}

template<class Interfaces>
auto port(Interfaces& interfaces, const std::string& joint, const char* field) {
    const std::string suffix = joint + "/" + field;
    using Pointer = decltype(&interfaces.front().get());
    Pointer found = nullptr;
    for (const auto& entry : interfaces) {
        const auto name = entry.get().get_name();
        if (name == suffix || (name.size() > suffix.size() &&
            name.compare(name.size() - suffix.size() - 1, suffix.size() + 1, "/" + suffix) == 0)) {
            if (found) throw std::runtime_error("Ambiguous ROS interface " + suffix);
            found = &entry.get();
        }
    }
    if (!found) throw std::runtime_error("Missing ROS interface " + suffix);
    return found;
}

void checkTensor(const Ort::TypeInfo& type, int64_t width) {
    if (type.GetONNXType() != ONNX_TYPE_TENSOR) throw std::runtime_error("Actor expects tensor I/O");
    const auto info = type.GetTensorTypeAndShapeInfo();
    if (info.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT ||
        info.GetShape() != std::vector<int64_t>{1, width}) throw std::runtime_error("Actor tensor shape/type mismatch");
}

std::string metadata(const Ort::ModelMetadata& meta, const char* key) {
    Ort::AllocatorWithDefaultOptions alloc;
    const auto value = meta.LookupCustomMetadataMapAllocated(key, alloc);
    return value ? value.get() : "";
}

double checked(double value) {
    if (!std::isfinite(value)) throw std::runtime_error("Nonfinite PPO feedback, reference, or actor value");
    return value;
}

class PlanarOdometryHold {
public:
    using Sample = std::array<float, 4>;  // x, y, vx, vy in the fixed entry-heading frame.

    struct ForwardParameters {
        float kp;
        float kd;
        float positionDeadband;
        float trimLimit;
        float positionCutoffHz;
        float velocityCutoffHz;
        float trimSlew;
    };

    void configureForward(ForwardParameters parameters) {
        const auto [kp, kd, positionDeadband, trimLimit, positionCutoffHz, velocityCutoffHz, trimSlew] = parameters;
        constexpr double nyquistHz = 0.5 / kPeriod;
        if (!std::isfinite(kp) || kp < 0.f || !std::isfinite(kd) || kd < 0.f ||
            !std::isfinite(positionDeadband) || positionDeadband < 0.f ||
            !std::isfinite(trimLimit) || trimLimit < 0.f || trimLimit > 0.3f ||
            !std::isfinite(positionCutoffHz) || positionCutoffHz <= 0.f || positionCutoffHz >= nyquistHz ||
            !std::isfinite(velocityCutoffHz) || velocityCutoffHz <= 0.f || velocityCutoffHz >= nyquistHz ||
            !std::isfinite(trimSlew) || trimSlew <= 0.f)
            throw std::runtime_error("Invalid key 9 forward hold parameters");
        const auto lowPassAlpha = [](double cutoffHz) {
            constexpr double pi = 3.14159265358979323846;
            const double sine = std::sin(pi * cutoffHz * kPeriod);
            // Set the digital one-pole filter's -3 dB frequency at 50 Hz sampling.
            return static_cast<float>(2.0 * sine / (std::sqrt(1.0 + sine * sine) + sine));
        };
        forwardPositionAlpha_ = lowPassAlpha(positionCutoffHz);
        forwardVelocityAlpha_ = lowPassAlpha(velocityCutoffHz);
        forwardMaxTrimStep_ = static_cast<float>(trimSlew * kPeriod);
        forward_ = parameters;
    }

    bool usesForwardHold() const { return forward_.has_value(); }

    void configureLateral(float positionGain, float positionDeadband, float trimLimit) {
        positionGain_[1] = positionGain;
        positionDeadband_[1] = positionDeadband;
        trimLimit_[1] = trimLimit;
    }

    void reset() {
        seedPending_ = true;
        active_ = false;
        nextSample_ = 0;
        entryTrim_.fill(0.f);
    }

    Eigen::Vector2d advance(const Sample& motion, bool active, double relativeYaw) {
        if (seedPending_) {
            previous_ = older_ = filtered_ = motion;
            if (!forward_) window_.fill(motion);
            seedPending_ = false;
        }
        Sample median{};
        for (std::size_t axis = 0; axis < motion.size(); ++axis) {
            median[axis] = std::clamp(motion[axis], std::min(previous_[axis], older_[axis]),
                                     std::max(previous_[axis], older_[axis]));
        }
        if (forward_) {
            for (std::size_t axis = 0; axis < motion.size(); ++axis) {
                const float alpha = axis < 2 ? forwardPositionAlpha_ : forwardVelocityAlpha_;
                filtered_[axis] += alpha * (median[axis] - filtered_[axis]);
            }
        } else {
            window_[nextSample_] = median;
            nextSample_ = (nextSample_ + 1) % window_.size();
            for (std::size_t axis = 0; axis < motion.size(); ++axis) {
                float sum = 0.f;
                for (const auto& value : window_) sum += value[axis];
                // At 50 Hz a 0.65-s gait period spans 32.5 samples.
                const float mean = (sum - 0.5f * window_[nextSample_][axis]) / 32.5f;
                filtered_[axis] += kFilterAlpha * (mean - filtered_[axis]);
            }
        }
        older_ = previous_;
        previous_ = motion;

        const auto c = static_cast<float>(std::cos(relativeYaw));
        const auto s = static_cast<float>(std::sin(relativeYaw));
        if (active && !active_) {
            anchor_ = {filtered_[0], filtered_[1]};
            if (forward_) forwardAxis_ = {c, s};
        }
        active_ = active;
        if (forward_) {
            float desired = 0.f;
            if (active) {
                const auto& hold = *forward_;
                const float positionError = forwardAxis_[0] * (anchor_[0] - filtered_[0]) +
                                            forwardAxis_[1] * (anchor_[1] - filtered_[1]);
                const float measuredSpeed = forwardAxis_[0] * filtered_[2] + forwardAxis_[1] * filtered_[3];
                const float alignment = c * forwardAxis_[0] + s * forwardAxis_[1];
                const float correction = hold.kp * deadband(positionError, hold.positionDeadband) -
                                         hold.kd * measuredSpeed;
                desired = std::clamp(alignment * correction, -hold.trimLimit, hold.trimLimit);
            }
            entryTrim_[0] += std::clamp(desired - entryTrim_[0], -forwardMaxTrimStep_, forwardMaxTrimStep_);
            return {entryTrim_[0], 0.0};
        }
        for (std::size_t axis = 0; axis < entryTrim_.size(); ++axis) {
            float desired = 0.f;
            if (active) {
                const float positionError = deadband(anchor_[axis] - filtered_[axis], positionDeadband_[axis]);
                const float speedError = deadband(filtered_[axis + 2], 0.008f);
                desired = std::clamp(positionGain_[axis] * positionError - 0.6f * speedError,
                                     -trimLimit_[axis], trimLimit_[axis]);
            }
            entryTrim_[axis] += std::clamp(desired - entryTrim_[axis], -kMaxTrimStep, kMaxTrimStep);
        }
        const float bodyX = c * entryTrim_[0] + s * entryTrim_[1];
        const float bodyY = -s * entryTrim_[0] + c * entryTrim_[1];
        return {bodyX, std::clamp(bodyY, -trimLimit_[1], trimLimit_[1])};
    }

private:
    static float deadband(float value, float threshold) {
        return std::copysign(std::max(0.f, std::abs(value) - threshold), value);
    }

    inline static const float kFilterAlpha = static_cast<float>(-std::expm1(-kPeriod / 0.25));
    static constexpr float kMaxTrimStep = static_cast<float>(0.025 * kPeriod);
    std::array<float, 2> positionGain_{0.35f, 0.35f};
    std::array<float, 2> positionDeadband_{0.015f, 0.015f};
    std::array<float, 2> trimLimit_{0.05f, 0.04f};
    std::optional<ForwardParameters> forward_;
    float forwardPositionAlpha_ = 0.f;
    float forwardVelocityAlpha_ = 0.f;
    float forwardMaxTrimStep_ = 0.f;
    std::array<float, 2> forwardAxis_{1.f, 0.f};

    Sample previous_{}, older_{}, filtered_{};
    std::array<Sample, 33> window_{};
    std::array<float, 2> anchor_{}, entryTrim_{};
    std::size_t nextSample_ = 0;
    bool seedPending_ = true;
    bool active_ = false;
};
}

struct FlatPpoTracker::Impl {
    struct Joint {
        hardware_interface::LoanedStateInterface *q, *dq;
        hardware_interface::LoanedCommandInterface *pos, *vel, *effort, *kp, *kd;
    };
    struct HybridMeasurement {
        const Eigen::Vector3d& position;
        const Eigen::Vector3d& velocity;
        double yaw;
        std::int64_t elapsed_ns;
        const WaveGenerator& wave;
    };
    CtrlInterfaces& io;
    const Mode mode;
    PolicySpec policy;
    std::array<Joint, 12> joints{};
    std::array<hardware_interface::LoanedStateInterface*, 7> imu{};
    std::array<double, 12> startPos{}, startKp{}, startKd{}, goal{};
    std::array<double, 12> firstPole{}, smoothedPos{};
    SmoothGaitReference smoothReference;
    LocalGaitReference localReference;
    std::array<float, kHybridVelocityWidth> obs{};
    std::array<float, 12> raw{};
    Ort::Env env{ORT_LOGGING_LEVEL_WARNING, "sysu219_mpc_ppo"};
    Ort::MemoryInfo memory = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    std::unique_ptr<Ort::Session> session;
    Ort::Value input{nullptr}, output{nullptr};
    bool started = false, fault = false, inferred = false;
    bool equal_hybrid_phases = false, hybrid_velocity = false, hybrid_filtered_odometry = false;
    bool hybrid_xy_hold = false, hybrid_velocity_only = false;
    double elapsed = 0., sinceInference = 0., blend = 0.;
    double sinceHostTick = 0.;
    std::int64_t nextHybridPolicyNs = 0, previousHybridPolicyNs = 0;
    std::int64_t nextHybridHostNs = 0, previousHybridHostNs = 0;
    Eigen::Vector3d hybridPositionOrigin = Eigen::Vector3d::Zero();
    Eigen::Vector2d hybridHeading = Eigen::Vector2d::UnitX();
    double hybridEntryYaw = 0.;
    std::array<float, 4> hybridOdometryFirstPole{}, hybridOdometrySecondPole{};
    std::array<float, 3> hybridFilteredVelocity{};
    PlanarOdometryHold planarHold;
    double transition, entryTolerance, entrySpeed, maxDt;

    Impl(CtrlInterfaces& interfaces, Mode selected, const std::filesystem::path& path)
        : io(interfaces), mode(selected), policy(policyFor(mode)) {
        if (!io.node) throw std::runtime_error("Missing ROS controller node");
        transition = checked(io.node->get_parameter("rl_pose_transition_s").as_double());
        entryTolerance = checked(io.node->get_parameter("rl_entry_tolerance_rad").as_double());
        entrySpeed = checked(io.node->get_parameter("rl_entry_velocity_rad_s").as_double());
        maxDt = checked(io.node->get_parameter("rl_max_control_period_s").as_double());
        if (transition <= 0. || entryTolerance <= 0. || entrySpeed <= 0. || maxDt < kPeriod)
            throw std::runtime_error("Invalid standing transition or timer settings");
        if (policy.pure && std::abs(transition - 0.5) > 1.e-9)
            throw std::runtime_error("Pure PPO was trained with a 0.5-s standing transition");
        for (std::size_t j = 0; j < joints.size(); ++j) {
            const std::string name = std::string(kLegs[j % 4]) + "_" + kKinds[j / 4] + "_joint";
            joints[j] = {port(io.joint_position_state_interface_, name, "position"),
                         port(io.joint_velocity_state_interface_, name, "velocity"),
                         port(io.joint_position_command_interface_, name, "position"),
                         port(io.joint_velocity_command_interface_, name, "velocity"),
                         port(io.joint_torque_command_interface_, name, "effort"),
                         port(io.joint_kp_command_interface_, name, "kp"),
                         port(io.joint_kd_command_interface_, name, "kd")};
        }
        const std::array<const char*, 7> fields{"orientation.w", "orientation.x", "orientation.y", "orientation.z",
                                                "angular_velocity.x", "angular_velocity.y", "angular_velocity.z"};
        const auto name = io.node->get_parameter("imu_name").as_string();
        for (std::size_t i = 0; i < fields.size(); ++i) imu[i] = port(io.imu_state_interface_, name, fields[i]);
        Ort::SessionOptions options;
        options.SetIntraOpNumThreads(1);
        options.SetInterOpNumThreads(1);
        options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_DISABLE_ALL);
        session = std::make_unique<Ort::Session>(env, path.c_str(), options);
        if (session->GetInputCount() != 1 || session->GetOutputCount() != 1)
            throw std::runtime_error("Actor must have a single input and output");
        checkTensor(session->GetOutputTypeInfo(0), 12);
        Ort::AllocatorWithDefaultOptions allocator;
        if (std::string(session->GetInputNameAllocated(0, allocator).get()) != "obs" ||
            std::string(session->GetOutputNameAllocated(0, allocator).get()) != "actions")
            throw std::runtime_error("Wrong actor tensor names");
        const auto meta = session->GetModelMetadata();
        const auto sha = metadata(meta, "sysu219.checkpoint_sha256");
        const auto fingerprint = metadata(meta, "sysu219.contract_sha256");
        const auto contract = metadata(meta, "sysu219.contract_id");
        hybrid_velocity = mode == Mode::HybridGait && contract == kHybridVelocityGaitId;
        hybrid_velocity_only = mode == Mode::HybridGait && contract == kHybridVelocityOnlyId;
        hybrid_xy_hold = mode == Mode::HybridGait && contract == kHybridXYHoldId;
        hybrid_filtered_odometry = hybrid_xy_hold || (mode == Mode::HybridGait && contract == kHybridFilteredOdometryId);
        const bool lateralObservationTrial = hybrid_xy_hold && fingerprint == kHybridXYHoldObservationTrialContract;
        if (hybrid_filtered_odometry) {
            policy.id = hybrid_xy_hold ? kHybridXYHoldId : kHybridFilteredOdometryId;
            policy.fingerprint = hybrid_xy_hold ? kHybridXYHoldContract : kHybridFilteredOdometryContract;
            if (lateralObservationTrial) policy.fingerprint = kHybridXYHoldObservationTrialContract;
            policy.width = kHybridFilteredOdometryWidth;
        } else if (hybrid_velocity_only) {
            policy.id = kHybridVelocityOnlyId;
            policy.fingerprint = kHybridVelocityOnlyContract;
            policy.width = kLocalGaitWidth + 3;
        } else if (hybrid_velocity) {
            policy.id = kHybridVelocityGaitId;
            policy.fingerprint = kHybridVelocityGaitContract;
            policy.width = kHybridVelocityWidth;
        }
        const auto diagonal_fingerprint = (hybrid_filtered_odometry || hybrid_velocity_only) ? policy.fingerprint
            : hybrid_velocity ? kDiagonalHybridVelocityGaitContract : kDiagonalHybridGaitContract;
        equal_hybrid_phases = mode == Mode::HybridGait && fingerprint == diagonal_fingerprint;
        if (contract != policy.id ||
            (fingerprint != policy.fingerprint && !equal_hybrid_phases) ||
            sha.size() != 64 || sha.find_first_not_of("0123456789abcdef") != std::string::npos ||
            (mode == Mode::FlatMpc && sha != kFlatCheckpoint))
            throw std::runtime_error("Wrong SYSU219 deployment contract or checkpoint");
        if (lateralObservationTrial) {
            planarHold.configureLateral(0.50f, 0.015f, 0.06f);
        } else if (hybrid_xy_hold && path.filename() == "pure_hybrid_gait_key6.onnx") {
            planarHold.configureLateral(2.50f, 0.01f, 0.06f);
        } else if (equal_hybrid_phases && sha == kKey9Checkpoint && path.filename() == "pure_hybrid_gait_key9.onnx") {
            const PlanarOdometryHold::ForwardParameters hold{
                static_cast<float>(io.node->get_parameter("rl_key9_hold_kp").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_kd").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_position_deadband_m").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_max_trim_m_s").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_position_cutoff_hz").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_velocity_cutoff_hz").as_double()),
                static_cast<float>(io.node->get_parameter("rl_key9_hold_trim_slew_m_s2").as_double())};
            planarHold.configureForward(hold);
            RCLCPP_INFO(io.node->get_logger(),
                        "Key 9 forward hold: Kp=%.3f Kd=%.3f deadband=%.3f m trim=%.3f m/s "
                        "pos_cutoff=%.1f Hz vel_cutoff=%.1f Hz slew=%.3f m/s^2",
                        hold.kp, hold.kd, hold.positionDeadband, hold.trimLimit,
                        hold.positionCutoffHz, hold.velocityCutoffHz, hold.trimSlew);
        }
        checkTensor(session->GetInputTypeInfo(0), policy.width);
        const std::array<int64_t, 2> inShape{1, policy.width}, outShape{1, 12};
        input = Ort::Value::CreateTensor<float>(memory, obs.data(), policy.width, inShape.data(), 2);
        output = Ort::Value::CreateTensor<float>(memory, raw.data(), raw.size(), outShape.data(), 2);
        infer();  // Initialize the execution path before entering TROTTING.
        raw.fill(0.f);
        RCLCPP_INFO(io.node->get_logger(), "PPO loaded %s, checkpoint %s", path.c_str(), sha.c_str());
    }

    void infer() {
        constexpr std::array<const char*, 1> inputs{"obs"}, outputs{"actions"};
        session->Run(Ort::RunOptions{nullptr}, inputs.data(), &input, 1, outputs.data(), &output, 1);
        for (float a : raw) checked(a);
    }

    void update(double dt, const Eigen::Vector3d* velocity, const Eigen::Vector3d& cmd,
                const Eigen::Matrix<double, 3, 4>& feet, const Eigen::Matrix<int, 4, 1>& contact, double gaitCycle,
                const SmoothGaitReference::Measurement* measurement = nullptr,
                const HybridMeasurement* hybrid = nullptr) {
        if (!std::isfinite(dt) || dt < 0. || dt > maxDt) throw std::runtime_error("Invalid control interval");
        inferred = false;
        if (dt == 0.) return;
        const bool local = mode == Mode::LocalGait || mode == Mode::HybridGait;
        if (!cmd.allFinite() || (!local && (!velocity || !velocity->allFinite())))
            throw std::runtime_error("Invalid policy velocity input");
        const bool smooth = mode == Mode::SmoothGait;
        if (smooth) {
            if (!measurement || !measurement->position_w.allFinite() || !measurement->velocity_w.allFinite() ||
                !measurement->feet_w.allFinite() || !std::isfinite(measurement->yaw) ||
                !std::isfinite(measurement->yaw_rate_w))
                throw std::runtime_error("Invalid estimated state for smooth PPO");
        }
        if (mode == Mode::HybridGait && (!hybrid || (!hybrid_velocity_only && !hybrid->position.allFinite()) ||
                ((hybrid_velocity || hybrid_velocity_only) && !hybrid->velocity.allFinite()) ||
                (hybrid_filtered_odometry && !hybrid->velocity.head<2>().allFinite()) ||
                !std::isfinite(hybrid->yaw) || hybrid->elapsed_ns < 0 ||
                (!started && hybrid->elapsed_ns != 0) ||
                (started && hybrid->elapsed_ns < previousHybridHostNs))) {
            throw std::runtime_error("Invalid hybrid odometry or shared gait time");
        }
        if (hybrid && !started) {
            const double period = equal_hybrid_phases ? WaveGenerator::kDiagonalTrotPeriod
                                                      : WaveGenerator::kLegacyTrotPeriod;
            const double stance_ratio = equal_hybrid_phases ? WaveGenerator::kDiagonalTrotStanceRatio
                                                            : WaveGenerator::kLegacyTrotStanceRatio;
            if (std::abs(hybrid->wave.getGaitPeriod() - period) > 1.e-12 ||
                std::abs(hybrid->wave.get_t_stance() - period * stance_ratio) > 1.e-12)
                throw std::runtime_error("Hybrid PPO clock disagrees with the deployed policy contract");
        }
        double hostInterval = kControllerPeriod;
        if (policy.filtered_targets) {
            if (hybrid && started) {
                if (hybrid->elapsed_ns < nextHybridHostNs) return;
                hostInterval = (hybrid->elapsed_ns - previousHybridHostNs) * kSecondsPerNanosecond;
            } else if (started) {
                sinceHostTick += dt;
                if (sinceHostTick + 1.e-12 < kControllerPeriod) return;
                // A faster caller can hold the last command between host ticks.
                // A late callback never sends several corrections using one sample.
                sinceHostTick = std::max(0., sinceHostTick - kControllerPeriod *
                    std::floor((sinceHostTick + 1.e-12) / kControllerPeriod));
            }
        }
        const auto& lower = policy.pure ? kGaitLow : kLow;
        const auto& upper = policy.pure ? kGaitHigh : kHigh;
        std::array<double, 12> q{}, dq{};
        for (std::size_t j = 0; j < joints.size(); ++j) {
            q[j] = checked(joints[j].q->get_value());
            dq[j] = checked(joints[j].dq->get_value());
            if (!policy.pure && std::abs(dq[j]) > 22.) throw std::runtime_error("PPO joint speed outside motor envelope");
            if (!started) {
                startPos[j] = checked(joints[j].pos->get_value());
                startKp[j] = checked(joints[j].kp->get_value());
                startKd[j] = checked(joints[j].kd->get_value());
                if (startKp[j] < 0. || startKd[j] < 0. || std::abs(q[j] - startPos[j]) > entryTolerance ||
                    std::abs(dq[j]) > entrySpeed || startPos[j] < lower[j / 4] || startPos[j] > upper[j / 4])
                    throw std::runtime_error("PPO requires settled standing feedback at entry");
                if (policy.filtered_targets) {
                    if (q[j] < lower[j / 4] || q[j] > upper[j / 4])
                        throw std::runtime_error("PPO standing feedback is outside joint bounds");
                    startPos[j] = firstPole[j] = smoothedPos[j] = q[j];
                }
            }
        }
        Eigen::Quaterniond rot(checked(imu[0]->get_value()), checked(imu[1]->get_value()),
                               checked(imu[2]->get_value()), checked(imu[3]->get_value()));
        if (!std::isfinite(rot.norm()) || rot.norm() < 1.e-6) throw std::runtime_error("Invalid IMU orientation");
        const Eigen::Vector3d gravity = rot.normalized().conjugate() * Eigen::Vector3d(0., 0., -1.);
        const bool lateralCommand = hybrid && hybrid_xy_hold && cmd.y() != 0.0;
        Eigen::Vector3d bounded = FlatPpoTracker::boundCommand(cmd);
        if (lateralCommand) {
            bounded << 0.0, std::clamp(cmd.y(), -kHybridLateralSpeedLimit, kHybridLateralSpeedLimit), 0.0;
        } else if (policy.filtered_targets) {
            bounded << std::clamp(cmd.x(), -0.3, 0.3), 0.0, std::clamp(cmd.z(), -0.3, 0.3);
        }
        const std::size_t feedbackOffset = local ? 0 : 3;
        for (std::size_t i = 0; i < 3; ++i) {
            if (!local) obs[i] = static_cast<float>((*velocity)[i]);
            obs[feedbackOffset + i] = static_cast<float>(checked(imu[4 + i]->get_value()));
            obs[feedbackOffset + 3 + i] = static_cast<float>(gravity[i]);
            obs[feedbackOffset + 6 + i] = static_cast<float>(bounded[i]);
        }
        for (std::size_t j = 0; j < joints.size(); ++j) {
            obs[feedbackOffset + 9 + j] = static_cast<float>(q[j] - kPose[j / 4]);
            obs[feedbackOffset + 21 + j] = static_cast<float>(dq[j]);
            obs[feedbackOffset + 33 + j] = raw[j];
        }
        if (mode == Mode::FootTrackingMpc) {
            if (!feet.allFinite()) throw std::runtime_error("Invalid MPC foot trajectory");
            for (int leg = 0; leg < 4; ++leg) {
                if (contact[leg] != 0 && contact[leg] != 1) throw std::runtime_error("Invalid planned contact");
                for (int axis = 0; axis < 3; ++axis) obs[48 + 3 * leg + axis] = static_cast<float>(feet(axis, leg));
                obs[60 + leg] = static_cast<float>(contact[leg]);
            }
        }
        if (mode == Mode::PureGait) {
            constexpr double tau = 6.2831853071795864769;
            const double phase = tau * std::remainder(checked(gaitCycle), 1.0);
            obs[48] = static_cast<float>(std::sin(phase));
            obs[49] = static_cast<float>(std::cos(phase));
        }
        if (hybrid) {
            elapsed = hybrid->elapsed_ns * kSecondsPerNanosecond;
            previousHybridHostNs = hybrid->elapsed_ns;
            nextHybridHostNs = (hybrid->elapsed_ns / kControllerPeriodNs + 1) * kControllerPeriodNs;
        } else if (started) {
            const double tick = policy.filtered_targets ? kControllerPeriod : dt;
            elapsed += tick;
            sinceInference += tick;
        }
        const bool inferenceDue = hybrid ? hybrid->elapsed_ns >= nextHybridPolicyNs
                                         : !started || sinceInference + 1.e-12 >= kPeriod;
        if (smooth && inferenceDue) {
            const auto reference = smoothReference.step(*measurement, bounded);
            std::copy(reference.begin(), reference.end(), obs.begin() + 48);
        } else if (hybrid && inferenceDue) {
            const auto current = hybrid->wave.sampleSynchronizedTrot(hybrid->elapsed_ns);
            const auto next = hybrid->wave.sampleSynchronizedTrot(hybrid->elapsed_ns + kPolicyPeriodNs);
            const double interval = started
                ? (hybrid->elapsed_ns - previousHybridPolicyNs) * kSecondsPerNanosecond : kPeriod;
            if (!hybrid_velocity_only) {
                if (!started) {
                    hybridPositionOrigin = hybrid->position;
                    hybridEntryYaw = hybrid->yaw;
                    hybridHeading << std::cos(hybrid->yaw), std::sin(hybrid->yaw);
                }
                const Eigen::Vector3d delta = hybrid->position - hybridPositionOrigin;
                obs[kLocalGaitWidth] = static_cast<float>(hybridHeading.dot(delta.head<2>()));
                obs[kLocalGaitWidth + 1] = static_cast<float>(-hybridHeading.y() * delta.x() + hybridHeading.x() * delta.y());
                if (!hybrid_filtered_odometry) obs[kLocalGaitWidth + 2] = static_cast<float>(delta.z());
                const bool forwardHold = planarHold.usesForwardHold();
                if (hybrid_filtered_odometry || forwardHold) {
                    Eigen::Vector2d v = hybrid->velocity.head<2>();
                    if (!v.allFinite()) v.setZero();
                    // The training filter sees both XY vectors in the same fixed entry-yaw frame.
                    const PlanarOdometryHold::Sample measurement{
                        obs[kLocalGaitWidth], obs[kLocalGaitWidth + 1],
                        static_cast<float>(hybridHeading.dot(v)),
                        static_cast<float>(-hybridHeading.y() * v.x() + hybridHeading.x() * v.y())};
                    if ((hybrid_xy_hold || forwardHold) && started) {
                        for (const float value : measurement) checked(value);
                        const bool idle = (cmd.array() == 0.).all() && (!forwardHold || elapsed >= transition);
                        const auto trim = planarHold.advance(measurement, idle, hybrid->yaw - hybridEntryYaw);
                        if (!lateralCommand) bounded.head<2>() += trim;
                        bounded.x() = std::clamp(bounded.x(), -0.3, 0.3);
                        obs[feedbackOffset + 6] = static_cast<float>(bounded.x());
                        obs[feedbackOffset + 7] = static_cast<float>(bounded.y());
                    }
                    if (hybrid_filtered_odometry) {
                        if (!started) {
                            hybridOdometryFirstPole = hybridOdometrySecondPole = measurement;
                        } else {
                            for (std::size_t i = 0; i < measurement.size(); ++i) {
                                auto& first = hybridOdometryFirstPole[i];
                                auto& second = hybridOdometrySecondPole[i];
                                first += kHybridOdometryAlpha * (measurement[i] - first);
                                second += kHybridOdometryAlpha * (first - second);
                            }
                        }
                        std::copy(hybridOdometrySecondPole.begin(), hybridOdometrySecondPole.end(), obs.begin() + kLocalGaitWidth);
                    }
                }
            }
            if (hybrid_velocity || hybrid_velocity_only) {
                // Match the actor's current odometry heading frame for XY and world Z.
                const double c = std::cos(hybrid->yaw), s = std::sin(hybrid->yaw);
                const auto& v = hybrid->velocity;
                const std::array<float, 3> measured{
                    static_cast<float>(c * v.x() + s * v.y()),
                    static_cast<float>(-s * v.x() + c * v.y()),
                    static_cast<float>(v.z())};
                if (hybrid_velocity_only) {
                    if (!started) {
                        hybridFilteredVelocity = measured;
                    } else {
                        for (std::size_t i = 0; i < measured.size(); ++i)
                            hybridFilteredVelocity[i] += kHybridVelocityOnlyAlpha * (measured[i] - hybridFilteredVelocity[i]);
                    }
                    std::copy(hybridFilteredVelocity.begin(), hybridFilteredVelocity.end(), obs.begin() + kLocalGaitWidth);
                } else {
                    std::copy(measured.begin(), measured.end(), obs.begin() + kHybridPositionWidth);
                }
            }
            const auto reference = localReference.stepSynchronized(bounded, current.cycle, next.contact, next.phase,
                interval, hybrid->wave.get_t_stance(), hybrid->wave.get_t_swing());
            std::copy(reference.begin(), reference.end(), obs.begin() + kProprioceptionWidth);
            previousHybridPolicyNs = hybrid->elapsed_ns;
            nextHybridPolicyNs = (hybrid->elapsed_ns / kPolicyPeriodNs + 1) * kPolicyPeriodNs;
        } else if (local && inferenceDue) {
            const auto reference = localReference.step(bounded);
            std::copy(reference.begin(), reference.end(), obs.begin() + kProprioceptionWidth);
        }
        for (int64_t i = 0; i < policy.width; ++i) checked(obs[i]);
        if (inferenceDue) {
            infer();
            inferred = true;
            sinceInference = std::max(0., sinceInference - kPeriod * std::floor((sinceInference + 1.e-12) / kPeriod));
            for (std::size_t j = 0; j < joints.size(); ++j)
                goal[j] = std::clamp(kPose[j / 4] + 0.25 * raw[j], lower[j / 4], upper[j / 4]);
        }
        // The 50-Hz policy holds the blended target across five main-control ticks.
        if (!policy.pure || inferred) {
            const double u = std::clamp(elapsed / transition, 0., 1.);
            blend = u * u * (3. - 2. * u);
        }
        struct Command { double pos, kp, kd; };
        std::array<Command, 12> commands{};
        for (std::size_t j = 0; j < joints.size(); ++j) {
            double pos = startPos[j] + blend * (goal[j] - startPos[j]);
            if (policy.filtered_targets) {
                const double ratio = hostInterval / kTargetTimeConstant;
                const double decay = std::exp(-ratio);
                const double coupling = ratio * decay;
                const double alpha = -std::expm1(-ratio);
                // Exact hold discretization of the two equal continuous poles.
                smoothedPos[j] = decay * smoothedPos[j] + coupling * firstPole[j] + (alpha - coupling) * pos;
                firstPole[j] += alpha * (pos - firstPole[j]);
                pos = smoothedPos[j];
            }
            const double targetKp = policy.pure ? 260.0 : kKp[j / 4];
            const double targetKd = policy.pure ? 3.8 : 4.5;
            const double kp = startKp[j] + blend * (targetKp - startKp[j]);
            const double kd = startKd[j] + blend * (targetKd - startKd[j]);
            const double tau = checked(kp * (pos - q[j]) - kd * dq[j]);
            // Preserve the held joint goal. Scale both PD gains only if the
            // estimated joint effort would exceed the URDF or training speed curve.
            // Beyond no-load speed the motor may brake, but cannot drive further.
            const double lo = std::clamp(200. * (-1. - dq[j] / 22.), -100., 0.);
            const double hi = std::clamp(200. * (1. - dq[j] / 22.), 0., 100.);
            const double scale = tau == 0. ? 1. : std::clamp(tau, lo, hi) / tau;
            commands[j] = {checked(pos), checked(kp * scale), checked(kd * scale)};
        }
        // Commit only after validating all twelve outputs.
        for (std::size_t j = 0; j < joints.size(); ++j) {
            joints[j].pos->set_value(commands[j].pos);
            joints[j].vel->set_value(0.);
            joints[j].kp->set_value(commands[j].kp);
            joints[j].kd->set_value(commands[j].kd);
            joints[j].effort->set_value(0.);
        }
        started = true;
    }
};

FlatPpoTracker::FlatPpoTracker(CtrlInterfaces& interfaces, Mode mode, const std::string& model_name) {
    try {
        const auto path = std::filesystem::path(ament_index_cpp::get_package_share_directory("sysu219_guide_controller"))
            / "model" / (model_name.empty() ? std::string(policyFor(mode).model) : model_name);
        if (!model_name.empty() && !std::filesystem::is_regular_file(path)) {
            if (interfaces.node)
                RCLCPP_INFO(interfaces.node->get_logger(), "Optional PPO model unavailable: %s", path.c_str());
            return;
        }
        impl_ = std::make_unique<Impl>(interfaces, mode, path);
    } catch (const std::exception& e) {
        if (interfaces.node)
            RCLCPP_ERROR(interfaces.node->get_logger(), "PPO model %s unavailable: %s",
                         model_name.empty() ? "for the startup profile" : model_name.c_str(), e.what());
    }
}

Eigen::Vector3d FlatPpoTracker::boundCommand(const Eigen::Vector3d& command) {
    Eigen::Vector3d result = command;
    for (int i = 0; i < 3; ++i) {
        if (std::isfinite(command[i])) result[i] = std::clamp(command[i], -kCommandLimit[i], kCommandLimit[i]);
    }
    return result;
}

FlatPpoTracker::~FlatPpoTracker() = default;
bool FlatPpoTracker::ready() const { return impl_ && !impl_->fault; }
bool FlatPpoTracker::didInfer() const { return ready() && impl_->inferred; }
bool FlatPpoTracker::usesEqualHybridPhases() const { return impl_ && impl_->equal_hybrid_phases; }
bool FlatPpoTracker::supportsHybridLateralCommand() const { return ready() && impl_->hybrid_xy_hold; }

void FlatPpoTracker::reset() {
    if (!impl_) return;
    impl_->started = false;
    impl_->fault = false;
    impl_->inferred = false;
    impl_->elapsed = impl_->sinceInference = impl_->sinceHostTick = impl_->blend = 0.;
    impl_->nextHybridPolicyNs = impl_->previousHybridPolicyNs = 0;
    impl_->nextHybridHostNs = impl_->previousHybridHostNs = 0;
    impl_->smoothReference.reset();
    impl_->localReference.reset();
    impl_->hybridOdometryFirstPole.fill(0.f);
    impl_->hybridOdometrySecondPole.fill(0.f);
    impl_->hybridFilteredVelocity.fill(0.f);
    impl_->planarHold.reset();
    impl_->raw.fill(0.f);
}

bool FlatPpoTracker::step(double dt, const Eigen::Vector3d& velocity, const Eigen::Vector3d& cmd,
                          const Eigen::Matrix<double, 3, 4>& feet, const Eigen::Matrix<int, 4, 1>& contacts, double gaitCycle) {
    if (!ready() || impl_->mode == Mode::LocalGait || impl_->mode == Mode::HybridGait) return false;
    try {
        impl_->update(dt, &velocity, cmd, feet, contacts, gaitCycle);
        return true;
    } catch (const std::exception& e) {
        impl_->fault = true;
        RCLCPP_ERROR(impl_->io.node->get_logger(), "PPO actuator fault: %s", e.what());
        return false;
    }
}

bool FlatPpoTracker::stepSmooth(double dt, const Eigen::Vector3d& velocity, const Eigen::Vector3d& cmd,
                                const SmoothGaitReference::Measurement& measurement) {
    if (!ready() || impl_->mode != Mode::SmoothGait) return false;
    try {
        impl_->update(dt, &velocity, cmd, Eigen::Matrix<double, 3, 4>::Zero(),
                      Eigen::Matrix<int, 4, 1>::Zero(), 0., &measurement);
        return true;
    } catch (const std::exception& e) {
        impl_->fault = true;
        RCLCPP_ERROR(impl_->io.node->get_logger(), "Smooth PPO actuator fault: %s", e.what());
        return false;
    }
}

bool FlatPpoTracker::stepLocal(double dt, const Eigen::Vector3d& cmd) {
    if (!ready() || impl_->mode != Mode::LocalGait) return false;
    try {
        impl_->update(dt, nullptr, cmd, Eigen::Matrix<double, 3, 4>::Zero(),
                      Eigen::Matrix<int, 4, 1>::Zero(), 0.);
        return true;
    } catch (const std::exception& e) {
        impl_->fault = true;
        RCLCPP_ERROR(impl_->io.node->get_logger(), "Local PPO actuator fault: %s", e.what());
        return false;
    }
}

bool FlatPpoTracker::stepHybrid(double dt, const Eigen::Vector3d& cmd, const Eigen::Vector3d& positionWorld,
                               const Eigen::Vector3d& velocityWorld, double yawWorld,
                               std::int64_t gaitElapsedNs, const WaveGenerator& wave) {
    if (!ready() || impl_->mode != Mode::HybridGait) return false;
    try {
        const Impl::HybridMeasurement measurement{positionWorld, velocityWorld, yawWorld, gaitElapsedNs, wave};
        impl_->update(dt, nullptr, cmd, Eigen::Matrix<double, 3, 4>::Zero(),
                      Eigen::Matrix<int, 4, 1>::Zero(), 0., nullptr, &measurement);
        return true;
    } catch (const std::exception& e) {
        impl_->fault = true;
        RCLCPP_ERROR(impl_->io.node->get_logger(), "Hybrid PPO actuator fault: %s", e.what());
        return false;
    }
}
