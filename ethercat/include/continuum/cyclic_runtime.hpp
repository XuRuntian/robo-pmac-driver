#pragma once
#include <cerrno>
#include <cstdint>
#include <sched.h>
#include <stdexcept>
#include <sys/mman.h>
#include <sys/resource.h>
#include <time.h>

namespace continuum {
class StableWindow {
    std::uint64_t duration_, since_ = 0;
    bool tracking_ = false;
public:
    explicit StableWindow(std::uint64_t duration_ns) : duration_(duration_ns) {}
    bool observe(std::uint64_t time_ns, bool valid) {
        if (!valid || (tracking_ && time_ns < since_)) { tracking_ = false; return false; }
        if (!tracking_) { since_ = time_ns; tracking_ = true; }
        return time_ns - since_ >= duration_;
    }
};
inline void runtime_check(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
inline std::uint64_t monotonic_ns() {
    timespec t{};
    runtime_check(clock_gettime(CLOCK_MONOTONIC, &t) == 0, "monotonic clock failed");
    return std::uint64_t(t.tv_sec) * 1000000000ULL + t.tv_nsec;
}
// Process-local changes only; restore before blocking cleanup/logging. No
// system-wide IRQ affinity, CPU isolation, kernel, service or governor changes.
class CyclicRuntime {
    cpu_set_t old_cpus_{};
    sched_param old_param_{};
    int old_policy_ = -1;
    bool affinity_changed_ = false, locked_ = false, policy_changed_ = false;
public:
    int cpu = -1;
    bool fifo = false, memory_locked = false;
    long minor_faults_before = 0, major_faults_before = 0;
    CyclicRuntime() = default;
    CyclicRuntime(const CyclicRuntime&) = delete;
    CyclicRuntime& operator=(const CyclicRuntime&) = delete;
    void start(bool realtime, int requested_cpu) {
        runtime_check(old_policy_ == -1, "cyclic runtime already started");
        old_policy_ = sched_getscheduler(0);
        runtime_check(old_policy_ >= 0 && sched_getparam(0, &old_param_) == 0
                      && sched_getaffinity(0, sizeof old_cpus_, &old_cpus_) == 0, "scheduler query failed");
        if (requested_cpu >= 0) {
            runtime_check(requested_cpu < CPU_SETSIZE && CPU_ISSET(requested_cpu, &old_cpus_),
                          "requested CPU is outside permitted affinity");
            cpu_set_t selected;
            CPU_ZERO(&selected);
            CPU_SET(requested_cpu, &selected);
            runtime_check(sched_setaffinity(0, sizeof selected, &selected) == 0, "CPU affinity failed");
            affinity_changed_ = true;
        }
        if (realtime) {
            runtime_check(mlockall(MCL_CURRENT | MCL_FUTURE) == 0, "mlockall failed; realtime start refused");
            locked_ = memory_locked = true;
            // Prefault a bounded stack region before entering FIFO scheduling.
            volatile unsigned char stack[128 * 1024];
            for (unsigned i = 0; i < sizeof stack; i += 4096) stack[i] = 0;
            sched_param priority{};
            priority.sched_priority = 60;
            runtime_check(sched_setscheduler(0, SCHED_FIFO, &priority) == 0, "SCHED_FIFO failed; realtime start refused");
            policy_changed_ = fifo = true;
        }
        cpu = sched_getcpu();
        rusage usage{};
        runtime_check(getrusage(RUSAGE_SELF, &usage) == 0, "rusage failed");
        minor_faults_before = usage.ru_minflt;
        major_faults_before = usage.ru_majflt;
    }
    void restore() noexcept {
        if (policy_changed_) { sched_setscheduler(0, old_policy_, &old_param_); policy_changed_ = false; }
        if (locked_) { munlockall(); locked_ = false; }
        if (affinity_changed_) { sched_setaffinity(0, sizeof old_cpus_, &old_cpus_); affinity_changed_ = false; }
    }
    ~CyclicRuntime() { restore(); }
};
} // namespace continuum
