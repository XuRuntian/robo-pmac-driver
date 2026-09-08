#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

// Transport-independent building blocks for OFFLINE validation. No master,
// thread, drive activation, homing or physical stop implementation lives here.
namespace continuum {
constexpr std::size_t axes = 5;
using Vector = std::array<double, axes>;

enum class DriveState {
    not_ready, disabled, ready, switched_on, enabled, quick_stop,
    fault_reaction, fault, unknown
};

inline DriveState decode_status(std::uint16_t word) noexcept {
    switch (word & 0x004f) {
    case 0x0000: return DriveState::not_ready;
    case 0x0040: return DriveState::disabled;
    case 0x000f: return DriveState::fault_reaction;
    case 0x0008: return DriveState::fault;
    }
    switch (word & 0x006f) {
    case 0x0021: return DriveState::ready;
    case 0x0023: return DriveState::switched_on;
    case 0x0027: return DriveState::enabled;
    case 0x0007: return DriveState::quick_stop;
    default: return DriveState::unknown;
    }
}

// Caller owns the reset pulse edge and must confirm fresh feedback separately.
// Quick stop/fault never automatically resumes the enable sequence.
inline std::uint16_t control_word(DriveState state, bool enable,
                                  bool reset_pulse = false) noexcept {
    if (!enable) return 0x0000;
    switch (state) {
    case DriveState::disabled: return 0x0006;
    case DriveState::ready: return 0x0007;
    case DriveState::switched_on:
    case DriveState::enabled: return 0x000f;
    case DriveState::fault: return reset_pulse ? 0x0080 : 0x0000;
    default: return 0x0000;
    }
}

inline std::int32_t position_count(double value) {
    if (!std::isfinite(value) || value < std::numeric_limits<std::int32_t>::min()
        || value > std::numeric_limits<std::int32_t>::max()) {
        throw std::invalid_argument("position outside signed 32-bit drive range");
    }
    return static_cast<std::int32_t>(std::llround(value));
}

struct Sample { Vector position{}, velocity{}; };
struct Segment {
    Vector position{};
    Vector velocity{};  // counts/second, NOT the PMAC counts/millisecond
    std::uint32_t ticks = 0;
    std::uint64_t sequence = 0;
};
struct Limits {
    Vector lower{}, upper{}, max_velocity{}, max_acceleration{};
};

// Cubic Hermite PVT interpolation, with extrema checked over the ENTIRE
// continuous segment. Reject overshoot rather than clipping individual samples.
class Cubic {
    Vector a_{}, b_{}, c_{}, d_{};
    double seconds_;
public:
    Cubic(const Sample& start, const Segment& end, double period_s, const Limits& limits)
        : seconds_(end.ticks * period_s) {
        if (!std::isfinite(period_s) || period_s <= 0 || !std::isfinite(seconds_)
            || seconds_ <= 0 || seconds_ > 0.5) {
            throw std::invalid_argument("invalid PVT duration (max 500 ms)");
        }
        for (std::size_t i = 0; i < axes; ++i) {
            const auto p0 = start.position[i], p1 = end.position[i];
            const auto v0 = start.velocity[i], v1 = end.velocity[i];
            for (auto x : {p0, p1, v0, v1, limits.lower[i], limits.upper[i],
                           limits.max_velocity[i], limits.max_acceleration[i]}) {
                if (!std::isfinite(x)) throw std::invalid_argument("non-finite PVT data");
            }
            if (limits.lower[i] >= limits.upper[i] || limits.max_velocity[i] <= 0
                || limits.max_acceleration[i] <= 0) {
                throw std::invalid_argument("invalid motion limits");
            }
            position_count(p0);
            position_count(p1);
            d_[i] = p0;
            c_[i] = v0 * seconds_;
            b_[i] = 3 * (p1 - p0) - 2 * c_[i] - v1 * seconds_;
            a_[i] = 2 * (p0 - p1) + c_[i] + v1 * seconds_;
            if (!std::isfinite(a_[i]) || !std::isfinite(b_[i]) || !std::isfinite(c_[i]))
                throw std::invalid_argument("PVT coefficient overflow");
            auto check_position = [&](double u) {
                if (u < 0 || u > 1) return;
                const auto p = ((a_[i] * u + b_[i]) * u + c_[i]) * u + d_[i];
                position_count(p);
                if (p < limits.lower[i] || p > limits.upper[i])
                    throw std::invalid_argument("PVT position limit/overshoot");
            };
            auto check_velocity = [&](double u) {
                if (u < 0 || u > 1) return;
                const auto v = (3 * a_[i] * u * u + 2 * b_[i] * u + c_[i]) / seconds_;
                if (!std::isfinite(v) || std::abs(v) > limits.max_velocity[i])
                    throw std::invalid_argument("PVT velocity limit");
            };
            check_position(0); check_position(1);
            // Solve the derivative quadratic using a cancellation-resistant form.
            const auto A = 3 * a_[i], B = 2 * b_[i], C = c_[i];
            if (A == 0) {
                if (B != 0) check_position(-C / B);
            } else {
                const auto discriminant = B * B - 4 * A * C;
                if (!std::isfinite(discriminant)) throw std::invalid_argument("PVT root overflow");
                if (discriminant >= 0) {
                    const auto q = -0.5 * (B + std::copysign(std::sqrt(discriminant), B));
                    if (q == 0) check_position(-B / (2 * A));
                    else { check_position(q / A); check_position(C / q); }
                }
            }
            check_velocity(0); check_velocity(1);
            if (a_[i] != 0) check_velocity(-b_[i] / (3 * a_[i]));
            for (auto u : {0.0, 1.0}) {
                const auto acc = (6 * a_[i] * u + 2 * b_[i]) / (seconds_ * seconds_);
                if (!std::isfinite(acc) || std::abs(acc) > limits.max_acceleration[i])
                    throw std::invalid_argument("PVT acceleration limit");
            }
        }
    }

    Sample at(double fraction) const {
        if (!std::isfinite(fraction) || fraction < 0 || fraction > 1)
            throw std::invalid_argument("PVT sample fraction outside [0,1]");
        Sample result;
        for (std::size_t i = 0; i < axes; ++i) {
            result.position[i] = ((a_[i] * fraction + b_[i]) * fraction + c_[i]) * fraction + d_[i];
            result.velocity[i] = (3 * a_[i] * fraction * fraction + 2 * b_[i] * fraction + c_[i]) / seconds_;
        }
        return result;
    }
};

// Single-threaded bounded FIFO for the offline runner. A future IPC producer
// must synchronize access; this type is deliberately not called lock-free/RT.
template<std::size_t Capacity>
class SegmentQueue {
    static_assert(Capacity > 0);
    std::array<Segment, Capacity> values_{};
    std::size_t read_ = 0, size_ = 0;
    std::uint64_t last_sequence_ = 0;
public:
    bool push(const Segment& segment) noexcept {
        if (size_ == Capacity || segment.sequence == 0 || segment.sequence <= last_sequence_)
            return false;
        values_[(read_ + size_) % Capacity] = segment;
        ++size_;
        last_sequence_ = segment.sequence;
        return true;
    }
    bool pop(Segment& segment) noexcept {
        if (!size_) return false;
        segment = values_[read_];
        read_ = (read_ + 1) % Capacity;
        --size_;
        return true;
    }
    std::size_t size() const noexcept { return size_; }
};

enum class StopReason { none, bus, feedback, mode, drive, heartbeat, limit, underflow, clock };
struct Feedback {
    std::array<std::uint16_t, axes> status{};
    std::array<std::int8_t, axes> mode{};
    bool link = false, operational = false, wkc_complete = false, fresh = false;
};

// Eligibility gate, NOT a physical stop routine. On denial, the eventual live
// backend must run its validated stop/watchdog path and flush queued targets.
class MotionGate {
    bool armed_ = false, started_ = false;
    std::uint64_t last_tick_ = 0, heartbeat_ = 0, timeout_;
    StopReason reason_ = StopReason::none;
    static StopReason feedback_reason(const Feedback& f) noexcept {
        if (!f.link || !f.operational || !f.wkc_complete) return StopReason::bus;
        if (!f.fresh) return StopReason::feedback;
        for (std::size_t i = 0; i < axes; ++i) {
            if (f.mode[i] != 8) return StopReason::mode;
            if (decode_status(f.status[i]) != DriveState::enabled) return StopReason::drive;
        }
        return StopReason::none;
    }
public:
    explicit MotionGate(std::uint64_t timeout_ticks) : timeout_(timeout_ticks) {
        if (!timeout_) throw std::invalid_argument("heartbeat timeout must be positive");
    }
    bool arm(const Feedback& f, std::uint64_t now) noexcept {
        if (reason_ != StopReason::none || armed_ || feedback_reason(f) != StopReason::none)
            return false;
        armed_ = true; started_ = false; last_tick_ = heartbeat_ = now;
        return true;
    }
    bool heartbeat(std::uint64_t now) noexcept {
        if (!armed_ || now < last_tick_ || now < heartbeat_ || now - heartbeat_ >= timeout_)
            return false;
        heartbeat_ = now;
        return true;
    }
    bool tick(const Feedback& f, std::uint64_t now, bool within_limits, bool needs_segment,
              bool segment_available) noexcept {
        if (!armed_) return false;
        auto reason = feedback_reason(f);
        if (now < last_tick_ || (started_ && now != last_tick_ + 1) || now < heartbeat_)
            reason = StopReason::clock;
        else if (now - heartbeat_ >= timeout_) reason = StopReason::heartbeat;
        else if (!within_limits) reason = StopReason::limit;
        else if (needs_segment && !segment_available) reason = StopReason::underflow;
        last_tick_ = now; started_ = true;
        if (reason != StopReason::none) { reason_ = reason; armed_ = false; }
        return armed_;
    }
    StopReason reason() const noexcept { return reason_; }
};
} // namespace continuum
