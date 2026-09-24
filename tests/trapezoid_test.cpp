// TrapezoidGenerator (wrapper over the vendored WPILib TrapezoidProfile). run_to_goal() checks the
// per-cycle limits on every positioning move; each test adds the one property it is named for.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>

#include "test_harness.hpp"
#include "viam/lib/trapezoid.hpp"

using ethercat::servo::TrapezoidGenerator;

namespace {

constexpr double kDt = 0.001;  // 1 kHz
constexpr double kEps = 1e-6;

// Arm a goal and run until idle (or a cycle cap); return cycles taken. Every cycle: |v| <= v_max,
// |dv| <= a*dt, and (monotone approach) the position never passes the goal.
int run_to_goal(TrapezoidGenerator& g, std::int32_t goal, double v_max, double a, bool monotone = true, int cap = 200000) {
    const std::int32_t start = g.position();
    const int dir = goal >= start ? 1 : -1;
    CHECK(g.set_goal(goal, v_max, a));
    double prev_v = g.velocity();
    std::int32_t prev_p = start;
    int n = 0;
    while (!g.idle() && n < cap) {
        const std::int32_t p = g.step(kDt);
        const double v = g.velocity();
        CHECK(std::fabs(v) <= v_max + kEps);
        CHECK(std::fabs(v - prev_v) <= a * kDt + kEps);
        if (monotone) {
            CHECK(dir * (static_cast<std::int64_t>(goal) - p) >= 0);
            CHECK(dir * (static_cast<std::int64_t>(p) - prev_p) >= 0);
        }
        prev_v = v;
        prev_p = p;
        ++n;
    }
    return n;
}

}  // namespace

TEST("a long move forms a trapezoid: honours the limits every cycle, lands exactly, takes the analytic time") {
    TrapezoidGenerator g;
    g.reseed(0);
    const int n = run_to_goal(g, 200000, 50000.0, 100000.0);
    CHECK(g.idle());
    CHECK_EQ(g.position(), std::int32_t{200000});
    // 2*(v/a) + (d - v^2/a)/v = 1.0 + (200000 - 25000)/50000 = 4.5 s -> 4500 cycles (a triangle would take 2.83 s)
    CHECK(n >= 4499 && n <= 4502);
}

TEST("a short move forms a triangle in either direction and lands exactly") {
    TrapezoidGenerator g;
    g.reseed(1000);
    const int n = run_to_goal(g, 1500, 50000.0, 100000.0);
    CHECK_EQ(g.position(), std::int32_t{1500});
    CHECK(n > 100 && n < 300);  // 2*sqrt(d/a) = 2*sqrt(500/1e5) = 0.141 s
    (void)run_to_goal(g, -30000, 20000.0, 40000.0);
    CHECK(g.idle());
    CHECK_EQ(g.position(), std::int32_t{-30000});
}

TEST("retargeting mid-move is continuous: no velocity jump, lands on the new goal") {
    TrapezoidGenerator g;
    g.reseed(0);
    CHECK(g.set_goal(100000, 50000.0, 100000.0));
    for (int i = 0; i < 800; ++i) {
        (void)g.step(kDt);
    }
    const double v_before = g.velocity();
    CHECK(v_before > 40000.0);
    (void)run_to_goal(g, -100000, 50000.0, 100000.0, /*monotone=*/false);  // reverse: decelerate first
    CHECK(g.idle());
    CHECK_EQ(g.position(), std::int32_t{-100000});
}

TEST("a goal just behind a moving axis: bounded overshoot (v^2/2a), then converge and land") {
    TrapezoidGenerator g;
    g.reseed(0);
    const double a = 100000.0;
    CHECK(g.set_goal(1000000, 50000.0, a));
    for (int i = 0; i < 500; ++i) {  // cruising at 50000 counts/s
        (void)g.step(kDt);
    }
    const double v = g.velocity();
    const std::int32_t p = g.position();
    const std::int32_t goal = p - 100;  // just behind
    CHECK(g.set_goal(goal, 50000.0, a));
    std::int32_t max_p = p;
    int n = 0;
    while (!g.idle() && n < 20000) {
        max_p = std::max(max_p, g.step(kDt));
        CHECK(std::fabs(g.velocity()) <= 50000.0 + kEps);
        ++n;
    }
    CHECK(g.idle());
    CHECK_EQ(g.position(), goal);
    const double stop_dist = v * v / (2.0 * a) + v * kDt;  // physics bound plus one cycle
    CHECK(static_cast<double>(max_p - p) <= stop_dist + 1.0);
}

TEST("a lowered v_max while over speed recovers at the acceleration limit and never exceeds the old speed") {
    TrapezoidGenerator g;
    g.reseed(0);
    CHECK(g.set_velocity(50000.0, 100000.0));
    for (int i = 0; i < 600; ++i) {
        (void)g.step(kDt);
    }
    CHECK(std::fabs(g.velocity() - 50000.0) < kEps);
    CHECK(g.set_goal(2000000, 20000.0, 100000.0));  // far goal, lower ceiling
    double prev_v = g.velocity();
    int n = 0;
    bool recovered = false;
    while (n < 2000) {
        (void)g.step(kDt);
        const double v = g.velocity();
        CHECK(v <= 50000.0 + kEps);
        CHECK(std::fabs(v - prev_v) <= 100000.0 * kDt + kEps);
        if (v <= 20000.0 + kEps) {
            recovered = true;
        }
        prev_v = v;
        ++n;
    }
    CHECK(recovered);
    CHECK(g.velocity() <= 20000.0 + kEps);
}

TEST("velocity mode ramps to the target speed and integrates position; stop() ramps to rest and holds") {
    TrapezoidGenerator g;
    g.reseed(0);
    CHECK(g.set_velocity(10000.0, 20000.0));  // 1 rev/s at 10000 cpr, reached in 0.5 s
    for (int i = 0; i < 500; ++i) {
        (void)g.step(kDt);
    }
    CHECK(std::fabs(g.velocity() - 10000.0) < kEps);
    const std::int32_t p1 = g.position();
    for (int i = 0; i < 1000; ++i) {
        (void)g.step(kDt);
    }
    CHECK(std::abs((g.position() - p1) - 10000) <= 1);  // 1 s at 1 rev/s
    g.stop(20000.0);
    int n = 0;
    while (!g.idle() && n < 5000) {
        (void)g.step(kDt);
        ++n;
    }
    CHECK(g.idle());
    CHECK(n >= 499 && n <= 501);
    const std::int32_t held = g.position();
    CHECK_EQ(g.goal(), held);
    for (int i = 0; i < 100; ++i) {
        CHECK_EQ(g.step(kDt), held);
    }
}

TEST("reseed puts the generator at rest; invalid limits are rejected and leave it unchanged") {
    TrapezoidGenerator g;
    CHECK(g.set_goal(5000, 1000.0, 1000.0));
    (void)g.step(kDt);
    g.reseed(7);
    CHECK(g.idle());
    CHECK_EQ(g.position(), std::int32_t{7});
    const double nan = std::nan("");
    CHECK(!g.set_goal(1000, nan, 100.0));
    CHECK(!g.set_goal(1000, 0.0, 100.0));
    CHECK(!g.set_goal(1000, -5.0, 100.0));
    CHECK(!g.set_goal(1000, 100.0, 0.0));
    CHECK(!g.set_goal(1000, 100.0, nan));
    CHECK(!g.set_velocity(nan, 100.0));
    CHECK(!g.set_velocity(10.0, -1.0));
    CHECK(g.idle());
    CHECK_EQ(g.step(kDt), std::int32_t{7});
    g.stop(nan);  // falls back to the last valid acceleration; already at rest
    CHECK(g.idle());
}

TEST("alternating near goals do not chatter or diverge") {
    TrapezoidGenerator g;
    g.reseed(1000);
    for (int i = 0; i < 400; ++i) {
        CHECK(g.set_goal((i % 2 == 0) ? 1000 : 1001, 5000.0, 50000.0));
        const std::int32_t p = g.step(kDt);
        CHECK(p >= 999 && p <= 1002);
        CHECK(std::fabs(g.velocity()) <= 200.0);
    }
}

TEST("the emitted position is a modular int32") {
    TrapezoidGenerator g;
    g.reseed(2147483000);
    CHECK(g.set_velocity(1000000.0, 1e12));  // instant ramp for the test
    std::int32_t last = 2147483000;
    bool wrapped = false;
    for (int i = 0; i < 5000; ++i) {
        const std::int32_t p = g.step(kDt);
        if (p < last) {
            wrapped = true;
        }
        last = p;
    }
    CHECK(wrapped);
    CHECK(last < 0);
}

TEST_MAIN()
