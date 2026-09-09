#include "continuum/commissioning_motion.hpp"
#include <iostream>

using namespace continuum;
void check(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }
CommissioningLimits limits() { return {90, 256, 64, 100, 1000000}; }
CommissioningFeedback initial() { return {1000000, 0x0240, -506000553, 0, 8, true, true, false}; }
void simulate(bool inject_fault, int displacement = 90) {
    auto bounded = limits(); bounded.displacement = displacement;
    CommissioningMotion motion(bounded);
    auto f = initial();
    bool enabled = false, disabled_after = false;
    auto low = f.position, high = f.position;
    for (unsigned i = 0; i < 12000; ++i) {
        if (inject_fault && i == 3500) f.status = 0x0208;
        const auto command = motion.step(f);
        check(command.control == 0 || command.control == 6 || command.control == 7 || command.control == 15,
              "unexpected controlword/reset");
        if (command.control == 15 && !enabled) {
            check(command.position == f.position, "first enable target was not aligned");
            enabled = true;
        }
        if (motion.phase() == CommissioningMotion::Phase::aborted) {
            check(inject_fault && i == 3500 && command.control == 0, "abort must disable immediately");
            f.status = 0x0240;
            f.time_ns += 1000000;
            check(motion.step(f).control == 0, "fault recovery re-enabled without a new procedure");
            return;
        }
        if (enabled && command.control == 0) disabled_after = true;
        f.position = command.position; // One-cycle ideal feedback, no mechanics simulated.
        low = std::min(low, f.position); high = std::max(high, f.position);
        if (command.control == 6) f.status = 0x0221;
        else if (command.control == 7) f.status = 0x0223;
        else if (command.control == 15) f.status = 0x0227;
        else f.status = 0x0240;
        f.time_ns += 1000000;
        if (motion.phase() == CommissioningMotion::Phase::done) {
            check(enabled && disabled_after && low == initial().position + std::min(0, displacement)
                  && high == initial().position + std::max(0, displacement),
                  "trajectory escaped bounds or enable/disable sequence incomplete");
            check(f.position == initial().position, "did not return to initial position");
            return;
        }
    }
    throw std::runtime_error("procedure failed to terminate");
}
int main() {
    simulate(false);
    simulate(false, -90);
    simulate(true);
    for (unsigned error = 0; error < 7; ++error) {
        CommissioningMotion m(limits());
        auto f = initial();
        m.step(f);
        f.time_ns += 1000000;
        if (error == 0) f.complete_wkc = false;
        if (error == 1) f.link_up = false;
        if (error == 2) f.mode = 1;
        if (error == 3) f.time_ns += 3000000;
        if (error == 4) f.position += 300;
        if (error == 5) f.torque = 101;
        if (error == 6) f.stop_requested = true;
        check(m.step(f).control == 0 && m.phase() == CommissioningMotion::Phase::aborted,
              "bad feedback/timing/limit/stop did not latch abort");
    }
    CommissioningMotion m(limits());
    auto f = initial(); f.status = 0x0027;
    check(m.step(f).control == 0 && m.phase() == CommissioningMotion::Phase::aborted,
          "accepted a pre-enabled drive");
    CommissioningMotion shifted(limits());
    f = initial(); shifted.step(f);
    // Slow drift before enabling must become the trajectory origin, avoiding
    // a jump back to the older preflight sample at the start of the ramp.
    for (unsigned i = 0; i < 1002; ++i) {
        f.time_ns += 1000000;
        f.position = initial().position + 10;
        auto command = shifted.step(f);
        if (command.control == 6) f.status = 0x0221;
        if (command.control == 7) f.status = 0x0223;
        if (command.control == 15) {
            check(command.position == f.position && shifted.anchor() == f.position,
                  "motion origin differs from first enable feedback");
            break;
        }
    }
    std::cout << "PASS: synthetic small-motion policy bounds, target alignment, state transitions and latched aborts\n";
}
