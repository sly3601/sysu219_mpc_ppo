//
// Created by biao on 24-9-18.
//


#ifndef WAVEGENERATOR_H
#define WAVEGENERATOR_H
#include <chrono>
#include <array>
#include <cstdint>
#include <vector>
#include <controller_common/common/enumClass.h>
#include <sysu219_guide_controller/common/mathTypes.h>

inline long long getSystemTime() {
    const auto now = std::chrono::system_clock::now();
    const auto duration = now.time_since_epoch();
    return std::chrono::duration_cast<std::chrono::microseconds>(duration).count();
}

class WaveGenerator {
public:
    static constexpr double kLegacyTrotPeriod = 0.975;
    static constexpr double kLegacyTrotStanceRatio = 2.0 / 3.0;
    static constexpr double kDiagonalTrotPeriod = 0.650;
    static constexpr double kDiagonalTrotStanceRatio = 0.5;

    struct TrotSample {
        double cycle;
        std::array<int, 4> contact;
        std::array<double, 4> phase;
    };

    WaveGenerator(double period, double st_ratio, const Vec4 &bias);

    ~WaveGenerator() = default;

    void startTrotFromStand();
    void update(double control_dt);
    // Hybrid PPO uses a timestamp relative to the first trotting frame.
    // Current and preview contacts are evaluated from the same half-cycle origin.
    [[nodiscard]] TrotSample sampleSynchronizedTrot(std::int64_t elapsed_ns) const;
    void updateSynchronizedTrot(std::int64_t elapsed_ns, double control_dt);
    // 第 0 行是当前接触；保持当前请求模式，在副本上推进实际切换规则。
    [[nodiscard]] std::vector<std::array<int, 4>> getMpcContactTable(
        int steps, double prediction_dt) const;

    [[nodiscard]] double get_t_stance() const { return period_ * st_ratio_; }
    [[nodiscard]] double get_t_swing() const { return period_ * (1 - st_ratio_); }
    [[nodiscard]] double getGaitPeriod() const { return period_; }
    [[nodiscard]] const VecInt4& getSwitchStatus() const { return switch_status_; }
    [[nodiscard]] double getPhaseControlTime() const { return control_time_; }
    [[nodiscard]] double getControlDt() const { return control_dt_; }

    Vec4 phase_;
    VecInt4 contact_;
    WaveStatus status_{};

private:
    /**
     * Update phase, contact and status based on current time.
     * @param phase foot phase
     * @param contact foot contact
     * @param status Wave Status
     */
    void calcWave(Vec4 &phase, VecInt4 &contact, WaveStatus status);

    double period_{};
    double st_ratio_{}; // stance phase ratio
    Vec4 bias_;

    Vec4 normal_t_ = Vec4::Zero(); // normalize time [0,1)
    Vec4 phase_past_; // foot phase
    VecInt4 contact_past_; // foot contact
    VecInt4 switch_status_;
    WaveStatus status_past_;

    double control_time_ = 0.0; // 步态相位时钟（秒），每次从站立进入踏步时对齐。
    double control_dt_ = 0.0;
};


#endif //WAVEGENERATOR_H
