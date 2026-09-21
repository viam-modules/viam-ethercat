#pragma once

// TrapezoidGenerator -- per-cycle set-point source for cyclic position (CSP) drives.
//
// Positioning uses the vendored WPILib TrapezoidProfile (src/third_party/wpilib, BSD-3, see
// THIRD_PARTY_NOTICES.md): a closed-form time-optimal trapezoid recomputed from the commanded state
// every cycle and sampled `dt` ahead, so a retarget is just a new goal and landing is exact. This
// wrapper adds a goal-less velocity run (set_rpm), a controlled stop, reseeding to the actual
// position before enable, limit validation, and the int32 wire value.
//
// A goal inside the stopping distance, or behind a moving axis, is passed by up to v^2/(2a) before
// the profile reverses (time-optimal, not overshoot-free). Units: counts and seconds. The output is
// modular int32 like 0x607A; a move must lie within +-2^31 counts of the current position.

#include <cmath>
#include <cstdint>

#include "third_party/wpilib/trapezoid_profile.hpp"

namespace ethercat::servo {

class TrapezoidGenerator {
   public:
    enum class Phase : std::uint8_t {
        Idle,         // holding position; v == 0
        Positioning,  // following the profile toward goal_
        Velocity,     // ramping to / running at v_target_ (no goal)
    };

    // Reseed to a known position at rest (before enable / after a fault) so the first streamed
    // target equals the actual position; a CSP drive enabled against a stale target lunges.
    void reseed(std::int32_t position) noexcept {
        state_ = {static_cast<double>(position), 0.0};
        goal_ = position;
        v_target_ = 0.0;
        phase_ = Phase::Idle;
    }

    // Move to an absolute goal (counts) at up to v_max counts/s with `accel` counts/s^2; a move in
    // flight is retargeted smoothly. False (and no change) unless both limits are finite and > 0.
    bool set_goal(std::int32_t goal, double v_max, double accel) noexcept {
        if (!valid_limit(v_max) || !valid_limit(accel)) {
            return false;
        }
        goal_ = goal;
        v_max_ = v_max;
        accel_ = accel;
        phase_ = Phase::Positioning;
        return true;
    }

    // Velocity run (no goal): ramp to v_target counts/s at `accel` and hold it; v_target == 0 ramps
    // to rest and holds position. False (and no change) on a non-finite v_target or invalid accel.
    bool set_velocity(double v_target, double accel) noexcept {
        if (!std::isfinite(v_target) || !valid_limit(accel)) {
            return false;
        }
        v_target_ = v_target;
        accel_ = accel;
        if (v_target == 0.0 && state_.velocity == 0.0) {
            goal_ = to_wire(state_.position);  // already at rest: a stop is a hold
            phase_ = Phase::Idle;
        } else {
            phase_ = Phase::Velocity;
        }
        return true;
    }

    // Decelerate to rest at `accel` (or the last valid acceleration when `accel` is invalid), then hold.
    void stop(double accel) noexcept {
        if (!set_velocity(0.0, accel)) {
            (void)set_velocity(0.0, accel_);
        }
    }

    // Advance one cycle of `dt` seconds and return the position to command this cycle.
    std::int32_t step(double dt) noexcept {
        if (phase_ == Phase::Idle || !(dt > 0.0)) {
            return to_wire(state_.position);
        }
        if (phase_ == Phase::Positioning) {
            wpilib::TrapezoidProfile profile({v_max_, accel_});
            const wpilib::TrapezoidProfile::State goal{static_cast<double>(goal_), 0.0};
            state_ = profile.Calculate(dt, state_, goal);
            // Calculate() returns the goal exactly once the remaining duration is shorter than dt.
            if (state_ == goal) {
                phase_ = Phase::Idle;
            }
            return to_wire(state_.position);
        }
        // Velocity run: constant-acceleration ramp toward v_target_, exact kinematics per step.
        const double dv_max = accel_ * dt;
        double dv = v_target_ - state_.velocity;
        if (dv > dv_max) {
            dv = dv_max;
        } else if (dv < -dv_max) {
            dv = -dv_max;
        }
        state_.position += state_.velocity * dt + 0.5 * dv * dt;
        state_.velocity += dv;  // lands exactly on v_target_ once within one step of it
        if (v_target_ == 0.0 && state_.velocity == 0.0) {
            goal_ = to_wire(state_.position);  // at rest: hold here (target() reports the hold point)
            phase_ = Phase::Idle;
        }
        return to_wire(state_.position);
    }

    Phase phase() const noexcept {
        return phase_;
    }
    bool idle() const noexcept {
        return phase_ == Phase::Idle;
    }
    std::int32_t goal() const noexcept {
        return goal_;
    }
    std::int32_t position() const noexcept {
        return to_wire(state_.position);
    }
    double velocity() const noexcept {
        return state_.velocity;
    }

   private:
    static bool valid_limit(double v) noexcept {
        return std::isfinite(v) && v > 0.0;
    }
    // Round to the nearest count as a two's-complement int32 (no UB on a value past the range).
    static std::int32_t to_wire(double p) noexcept {
        const auto r = static_cast<std::int64_t>(std::llround(p));
        return static_cast<std::int32_t>(static_cast<std::uint32_t>(static_cast<std::uint64_t>(r)));
    }

    wpilib::TrapezoidProfile::State state_{};  // commanded position (counts) and velocity (counts/s)
    double v_max_ = 1.0;
    double accel_ = 1.0;
    double v_target_ = 0.0;
    std::int32_t goal_ = 0;
    Phase phase_ = Phase::Idle;
};

}  // namespace ethercat::servo
