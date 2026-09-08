#include <algorithm>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <sched.h>
#include <stdexcept>
#include <string>
#include <time.h>
#include <vector>

static std::int64_t now_ns() {
    timespec ts{};
    if (clock_gettime(CLOCK_MONOTONIC, &ts)) throw std::runtime_error("clock_gettime failed");
    return ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

int main(int argc, char** argv) try {
    if (argc > 2) throw std::invalid_argument("usage: host_timing_probe [sample-count]");
    std::size_t count = 3000;
    if (argc == 2) {
        std::size_t used = 0;
        const auto value = std::stoll(argv[1], &used);
        if (used != std::string(argv[1]).size() || value < 100 || value > 10000)
            throw std::invalid_argument("sample-count must be 100..10000");
        count = static_cast<std::size_t>(value);
    }
    constexpr std::int64_t period = 1000000;
    std::vector<std::int64_t> lateness; lateness.reserve(count);
    std::uint64_t skipped = 0;
    auto deadline = now_ns() + period;
    const auto started = now_ns();
    for (std::size_t i = 0; i < count; ++i) {
        timespec ts{deadline / 1000000000, deadline % 1000000000};
        int result;
        do { result = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr); }
        while (result == EINTR);
        if (result) throw std::runtime_error("clock_nanosleep failed");
        const auto late = std::max<std::int64_t>(0, now_ns() - deadline);
        lateness.push_back(late);
        const auto missed = late / period;
        skipped += static_cast<std::uint64_t>(missed);
        deadline += (missed + 1) * period; // no burst of catch-up iterations
    }
    const auto elapsed = now_ns() - started;
    std::sort(lateness.begin(), lateness.end());
    const auto p99 = lateness[static_cast<std::size_t>(0.99 * (count - 1))];
    std::cout << "{\n  \"kind\": \"empty_loop_host_baseline_only\",\n"
              << "  \"period_ns\": " << period << ",\n  \"samples\": " << count
              << ",\n  \"scheduler_policy\": " << sched_getscheduler(0)
              << ",\n  \"median_lateness_ns\": " << lateness[count/2]
              << ",\n  \"p99_lateness_ns\": " << p99
              << ",\n  \"max_lateness_ns\": " << lateness.back()
              << ",\n  \"skipped_periods\": " << skipped
              << ",\n  \"elapsed_ns\": " << elapsed
              << ",\n  \"ethercat_or_realtime_acceptance\": false\n}\n";
} catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
