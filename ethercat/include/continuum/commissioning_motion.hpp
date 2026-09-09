#pragma once
#include "continuum/offline_core.hpp"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace continuum {
// Bounded commissioning policy. Actual use requires verified drive limits and
// an explicit mechanical envelope; this is not a physical emergency stop.
struct CommissioningLimits {
    std::int32_t displacement;
    std::int32_t travel_bound;
    std::int32_t following_bound;
    unsigned torque_bound;
    std::uint64_t period_ns;
};
struct CommissioningFeedback {
    std::uint64_t time_ns;
    std::uint16_t status;
    std::int32_t position;
    std::int16_t torque;
    int mode;
    bool complete_wkc, link_up, stop_requested;
};
struct CommissioningCommand {
    std::uint16_t control;
    std::int32_t position;
};
class CommissioningMotion {
public:
    enum class Phase { settle, shutdown, switch_on, enable, hold_before,
                       outward, hold_out, returning, hold_end, disable, done, aborted };
    enum class Failure { none, feedback, timing, drive_state, position, torque, timeout, requested_stop };
private:
    CommissioningLimits limits_;
    Phase phase_ = Phase::settle;
    Failure failure_ = Failure::none;
    bool started_ = false;
    std::uint64_t previous_time_ = 0, phase_start_ = 0;
    std::int32_t anchor_ = 0, target_ = 0;
    static constexpr std::uint64_t second = 1000000000ULL;
    static std::int64_t magnitude(std::int64_t value) { return value < 0 ? -value : value; }
    void enter(Phase phase, std::uint64_t time) { phase_ = phase; phase_start_ = time; }
    CommissioningCommand abort(Failure failure) {
        phase_ = Phase::aborted;
        failure_ = failure;
        return {0, target_};
    }
public:
    explicit CommissioningMotion(CommissioningLimits limits) : limits_(limits) {
        // This policy deliberately cannot represent an arbitrary jog/trajectory.
        if (!limits.displacement || magnitude(limits.displacement) > 128
                || limits.travel_bound <= magnitude(limits.displacement)
                || limits.travel_bound > 512 || limits.following_bound <= 0
                || limits.following_bound > 128 || !limits.torque_bound || limits.torque_bound > 100
                || (limits.period_ns != 1000000 && limits.period_ns != 2000000))
            throw std::invalid_argument("invalid bounded commissioning limits");
    }
    Phase phase() const { return phase_; }
    Failure failure() const { return failure_; }
    std::int32_t anchor() const { return anchor_; }
    CommissioningCommand step(const CommissioningFeedback& f) {
        if (phase_ == Phase::aborted || phase_ == Phase::done) return {0, target_};
        if (f.stop_requested) return abort(Failure::requested_stop);
        if (!f.complete_wkc || !f.link_up || f.mode != 8) return abort(Failure::feedback);
        if (started_ && (f.time_ns <= previous_time_ || f.time_ns - previous_time_ > 2 * limits_.period_ns))
            return abort(Failure::timing);
        const auto state = decode_status(f.status);
        if (state == DriveState::fault || state == DriveState::fault_reaction
                || state == DriveState::unknown || state == DriveState::quick_stop)
            return abort(Failure::drive_state);
        if (!started_) {
            if (state != DriveState::disabled) return abort(Failure::drive_state);
            const auto p = std::int64_t(f.position);
            if (p - limits_.travel_bound < std::numeric_limits<std::int32_t>::min()
                    || p + limits_.travel_bound > std::numeric_limits<std::int32_t>::max())
                return abort(Failure::position);
            anchor_ = target_ = f.position;
            phase_start_ = f.time_ns;
            started_ = true;
        }
        previous_time_ = f.time_ns;
        if (magnitude(std::int64_t(f.position) - anchor_) > limits_.travel_bound
                || magnitude(std::int64_t(f.position) - target_) > limits_.following_bound)
            return abort(Failure::position);
        if (magnitude(f.torque) > limits_.torque_bound) return abort(Failure::torque);
        const auto elapsed = f.time_ns - phase_start_;
        switch (phase_) {
        case Phase::settle:
            if (state != DriveState::disabled) return abort(Failure::drive_state);
            target_ = f.position;
            if (elapsed >= second) { enter(Phase::shutdown, f.time_ns); return {6, target_}; }
            return {0, target_};
        case Phase::shutdown:
            target_ = f.position;
            if (state == DriveState::ready) { enter(Phase::switch_on, f.time_ns); return {7, target_}; }
            if (state != DriveState::disabled || elapsed > second) return abort(Failure::timeout);
            return {6, target_};
        case Phase::switch_on:
            target_ = f.position;
            if (state == DriveState::switched_on) {
                // Target exactly matches feedback on the first enable frame.
                const auto p = std::int64_t(f.position);
                if (p - limits_.travel_bound < std::numeric_limits<std::int32_t>::min()
                        || p + limits_.travel_bound > std::numeric_limits<std::int32_t>::max())
                    return abort(Failure::position);
                anchor_ = f.position;
                enter(Phase::enable, f.time_ns);
                return {15, target_};
            }
            if (state != DriveState::ready || elapsed > second) return abort(Failure::timeout);
            return {7, target_};
        case Phase::enable:
            if (state == DriveState::enabled) { enter(Phase::hold_before, f.time_ns); return {15, target_}; }
            if (state != DriveState::switched_on || elapsed > second) return abort(Failure::timeout);
            return {15, target_};
        case Phase::hold_before:
            if (state != DriveState::enabled) return abort(Failure::drive_state);
            if (elapsed >= second) enter(Phase::outward, f.time_ns);
            return {15, target_};
        case Phase::outward:
        case Phase::returning: {
            if (state != DriveState::enabled) return abort(Failure::drive_state);
            const bool outward = phase_ == Phase::outward;
            const auto u = std::min(1.0, double(elapsed) / (2 * second));
            const auto blend = u * u * (3 - 2 * u);
            const auto delta = std::llround(limits_.displacement * (outward ? blend : 1 - blend));
            target_ = static_cast<std::int32_t>(std::int64_t(anchor_) + delta);
            if (elapsed >= 2 * second) enter(outward ? Phase::hold_out : Phase::hold_end, f.time_ns);
            return {15, target_};
        }
        case Phase::hold_out:
        case Phase::hold_end:
            if (state != DriveState::enabled) return abort(Failure::drive_state);
            if (elapsed >= second) {
                if (magnitude(std::int64_t(f.position) - target_) > std::min(16, limits_.following_bound))
                    return abort(Failure::position);
                if (phase_ == Phase::hold_out) enter(Phase::returning, f.time_ns);
                else { enter(Phase::disable, f.time_ns); return {0, target_}; }
            }
            return {15, target_};
        case Phase::disable:
            if (state == DriveState::disabled) { enter(Phase::done, f.time_ns); return {0, target_}; }
            if (elapsed > second) return abort(Failure::timeout);
            return {0, target_};
        case Phase::done:
        case Phase::aborted: return {0, target_};
        }
        return abort(Failure::drive_state);
    }
};
} // namespace continuum
