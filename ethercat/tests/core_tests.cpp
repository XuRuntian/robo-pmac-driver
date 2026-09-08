#include "continuum/offline_core.hpp"
#include "continuum/pdo.hpp"
#include <functional>
#include <iostream>
#include <string>
#include <vector>

using namespace continuum;
static void check(bool ok) { if (!ok) throw std::runtime_error("check failed"); }
static void near(double a, double b) { check(std::abs(a-b) < 1e-7); }
template<class Fn> static void rejects(Fn fn) {
    try { fn(); } catch (const std::invalid_argument&) { return; }
    throw std::runtime_error("invalid command accepted");
}
static Limits limits() {
    Limits l;
    l.lower.fill(-10000); l.upper.fill(10000);
    l.max_velocity.fill(100000); l.max_acceleration.fill(10000000);
    return l;
}
static Segment segment(std::uint64_t sequence = 1) {
    Segment s; s.position.fill(10); s.ticks = 20; s.sequence = sequence; return s;
}
static Feedback feedback() {
    Feedback f; f.status.fill(0x0237); f.mode.fill(8);
    f.link = f.operational = f.wkc_complete = f.fresh = true; return f;
}

int main() {
    const std::vector<std::pair<std::string, std::function<void()>>> tests{
        {"statusword_masks_and_optional_bits", [] {
            const std::array<std::pair<std::uint16_t, DriveState>, 8> cases{{
                {0, DriveState::not_ready}, {0x40, DriveState::disabled},
                {0x21, DriveState::ready}, {0x23, DriveState::switched_on},
                {0x27, DriveState::enabled}, {7, DriveState::quick_stop},
                {15, DriveState::fault_reaction}, {8, DriveState::fault}}};
            for (const auto& [word, expected] : cases)
                for (auto ignored : {0x0000, 0x0010, 0x0080, 0xff90})
                    check(decode_status(word | ignored) == expected);
            check(decode_status(0x65) == DriveState::unknown);
        }},
        {"controlword_enable_and_explicit_fault_reset", [] {
            check(control_word(DriveState::disabled, true) == 6);
            check(control_word(DriveState::ready, true) == 7);
            check(control_word(DriveState::switched_on, true) == 15);
            check(control_word(DriveState::enabled, false) == 0);
            check(control_word(DriveState::fault, true) == 0);
            check(control_word(DriveState::fault, true, true) == 128);
            check(control_word(DriveState::fault, false, true) == 0);
            check(control_word(DriveState::quick_stop, true) == 0);
            check(control_word(DriveState::unknown, true) == 0);
        }},
        {"pdo_little_endian_signed_and_unaligned", [] {
            std::array<std::uint8_t, 15> bytes{}; bytes.fill(0xa5);
            write_i32(bytes.data()+3, -2147483647-1);
            check(bytes[2] == 0xa5 && bytes[7] == 0xa5);
            check(bytes[3] == 0 && bytes[4] == 0 && bytes[5] == 0 && bytes[6] == 0x80);
            check(read_i32(bytes.data()+3) == std::numeric_limits<std::int32_t>::min());
            write_i32(bytes.data()+3, -1234567); check(read_i32(bytes.data()+3) == -1234567);
            write_u16(bytes.data()+1, 0x0237); check(bytes[1] == 0x37 && bytes[2] == 2);
            check(read_u16(bytes.data()+1) == 0x0237);
            write_u32(bytes.data()+10, 0x80000001); check(read_u32(bytes.data()+10) == 0x80000001);
            check(bytes[9] == 0xa5 && bytes[14] == 0xa5);
        }},
        {"signed_count_range_and_rounding", [] {
            check(position_count(-1.5) == -2); check(position_count(1.5) == 2);
            check(position_count(2147483647.0) == 2147483647);
            check(position_count(-2147483648.0) == std::numeric_limits<std::int32_t>::min());
            for (auto v : {2147483648.0, -2147483649.0, std::nan(""), double(INFINITY)})
                rejects([&] { position_count(v); });
        }},
        {"pvt_constant_velocity_units_are_counts_per_second", [] {
            Sample start; start.velocity.fill(500);
            auto end = segment(); end.velocity.fill(500);
            Cubic c(start, end, 0.001, limits());
            for (unsigned t = 0; t <= 20; ++t) {
                auto s = c.at(t/20.0);
                for (std::size_t i = 0; i < axes; ++i) { near(s.position[i], t*0.5); near(s.velocity[i], 500); }
            }
        }},
        {"pvt_five_axis_endpoints_and_zero_velocity_tail", [] {
            Sample start; start.position = {100, -200, 300, -400, 500};
            auto end = segment(); end.position = {120, -170, 340, -450, 560};
            Cubic c(start, end, 0.001, limits());
            for (std::size_t i = 0; i < axes; ++i) {
                near(c.at(0).position[i], start.position[i]); near(c.at(1).position[i], end.position[i]);
                near(c.at(0).velocity[i], 0); near(c.at(1).velocity[i], 0);
            }
            auto tail = end; tail.sequence = 2;
            Cubic hold(c.at(1), tail, 0.001, limits());
            for (std::size_t i = 0; i < axes; ++i) {
                near(hold.at(0.5).position[i], end.position[i]); near(hold.at(0.5).velocity[i], 0);
            }
        }},
        {"pvt_rejects_interior_position_overshoot", [] {
            Sample start; start.velocity.fill(1000);
            auto end = segment(); end.position.fill(0); end.velocity.fill(-1000);
            auto l = limits(); l.lower.fill(-4); l.upper.fill(4);
            rejects([&] { Cubic c(start, end, 0.001, l); });
        }},
        {"pvt_rejects_interior_velocity_peak", [] {
            auto l = limits(); l.max_velocity.fill(600);
            rejects([&] { Cubic c({}, segment(), 0.001, l); }); // peak 750, endpoint velocities 0
        }},
        {"pvt_rejects_acceleration_limit", [] {
            auto l = limits(); l.max_acceleration.fill(100000);
            rejects([&] { Cubic c({}, segment(), 0.001, l); }); // endpoint acceleration 150000
        }},
        {"pvt_rejects_duration_and_nonfinite_input", [] {
            for (auto ticks : {0u, 501u}) { auto s = segment(); s.ticks = ticks;
                rejects([&] { Cubic c({}, s, 0.001, limits()); }); }
            for (auto period : {0.0, -0.001, std::nan(""), double(INFINITY)})
                rejects([&] { Cubic c({}, segment(), period, limits()); });
            for (std::size_t i = 0; i < axes; ++i) {
                auto s = segment(); s.position[i] = std::nan("");
                rejects([&] { Cubic c({}, s, 0.001, limits()); });
                s = segment(); s.velocity[i] = INFINITY;
                rejects([&] { Cubic c({}, s, 0.001, limits()); });
            }
        }},
        {"pvt_rejects_invalid_limits_samples_and_raw_range", [] {
            auto l = limits(); l.max_velocity[3] = 0;
            rejects([&] { Cubic c({}, segment(), 0.001, l); });
            auto s = segment(); s.position[4] = 2147483648.0;
            rejects([&] { Cubic c({}, s, 0.001, limits()); });
            Cubic c({}, segment(), 0.001, limits());
            for (auto u : {-0.1, 1.01, std::nan("")}) rejects([&] { c.at(u); });
        }},
        {"queue_full_never_overwrites_and_rejected_token_can_retry", [] {
            SegmentQueue<2> q; check(q.push(segment(1))); check(q.push(segment(2)));
            check(!q.push(segment(3))); check(q.size() == 2);
            Segment s; check(q.pop(s) && s.sequence == 1);
            check(q.push(segment(3))); check(q.pop(s) && s.sequence == 2);
            check(q.pop(s) && s.sequence == 3); check(!q.pop(s));
            check(!q.push(segment(3))); check(!q.push(segment(0)));
        }},
        {"queue_wrap_preserves_order_and_all_axes", [] {
            SegmentQueue<3> q;
            for (std::uint64_t i = 1; i <= 1000; ++i) {
                auto in = segment(i); in.position[4] = double(i); check(q.push(in));
                Segment out; check(q.pop(out)); check(out.sequence == i); near(out.position[4], double(i));
            }
        }},
        {"gate_requires_explicit_arm_and_fresh_five_axis_feedback", [] {
            MotionGate g(500); auto f = feedback(); check(!g.tick(f, 0, true, false, false));
            f.fresh = false; check(!g.arm(f, 0)); f.fresh = true;
            f.mode[4] = 0; check(!g.arm(f, 0)); f.mode[4] = 8;
            f.status[3] = 0x23; check(!g.arm(f, 0)); f.status[3] = 0x27;
            check(g.arm(f, 0)); check(!g.arm(f, 0)); check(g.tick(f, 0, true, false, false));
        }},
        {"gate_bus_feedback_mode_fault_and_quickstop_latch", [] {
            for (int failure = 0; failure < 8; ++failure) {
                MotionGate g(500); auto f = feedback(); check(g.arm(f, 0));
                switch (failure) {
                case 0: f.link = false; break;
                case 1: f.operational = false; break;
                case 2: f.wkc_complete = false; break;
                case 3: f.fresh = false; break;
                case 4: f.mode[4] = 9; break;
                case 5: f.status[2] = 8; break;
                case 6: f.status[3] = 7; break;
                case 7: f.status[0] = 0x65; break;
                }
                check(!g.tick(f, 1, true, true, true));
                check(g.reason() != StopReason::none);
                check(!g.arm(feedback(), 2)); check(!g.tick(feedback(), 2, true, true, true));
            }
        }},
        {"gate_heartbeat_expiry_and_late_heartbeat_cannot_revive", [] {
            MotionGate g(3); auto f = feedback(); check(g.arm(f, 10));
            check(g.tick(f, 10, true, false, false)); check(g.tick(f, 11, true, false, false));
            check(g.tick(f, 12, true, false, false)); check(!g.heartbeat(13));
            check(!g.tick(f, 13, true, false, false)); check(g.reason() == StopReason::heartbeat);
            check(!g.heartbeat(14));
        }},
        {"gate_heartbeat_refresh", [] {
            MotionGate g(3); auto f = feedback(); check(g.arm(f, 0));
            for (std::uint64_t i = 0; i < 100; ++i) {
                if (i % 2 == 0) check(g.heartbeat(i));
                check(g.tick(f, i, true, false, false));
            }
        }},
        {"gate_underflow_and_limits", [] {
            auto f = feedback(); MotionGate g(500); check(g.arm(f, 0));
            check(!g.tick(f, 0, true, true, false)); check(g.reason() == StopReason::underflow);
            MotionGate h(500); check(h.arm(f, 0)); check(!h.tick(f, 0, false, false, false));
            check(h.reason() == StopReason::limit);
        }},
        {"gate_missed_duplicate_or_reversed_cycle", [] {
            for (auto bad : {9u, 10u, 12u}) {
                MotionGate g(500); auto f = feedback(); check(g.arm(f, 10));
                check(g.tick(f, 10, true, false, false));
                check(!g.tick(f, bad, true, false, false)); check(g.reason() == StopReason::clock);
            }
        }},
    };
    unsigned failures = 0;
    for (const auto& [name, test] : tests) {
        try { test(); std::cout << "PASS " << name << '\n'; }
        catch (const std::exception& e) { ++failures; std::cerr << "FAIL " << name << ": " << e.what() << '\n'; }
    }
    std::cout << tests.size() - failures << '/' << tests.size() << " offline core cases passed\n";
    return failures ? 1 : 0;
}
