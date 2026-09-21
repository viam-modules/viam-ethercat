// ============================================================================================
// THIRD-PARTY CODE -- WPILib (FIRST Robotics Competition), BSD 3-Clause license.
//
// Copyright (c) 2009-2026 FIRST and other WPILib contributors
// All rights reserved.
// Open Source Software; you can modify and/or share it under the terms of the WPILib BSD
// license, reproduced in full in src/third_party/wpilib/LICENSE.md.
//
// Derived from allwpilib: wpimath/src/main/native/include/wpi/math/trajectory/TrapezoidProfile.hpp
// (https://github.com/wpilibsuite/allwpilib). Modifications for this project:
//   * the wpi::units dimensional types are replaced by plain doubles (positions in counts, time
//     in seconds), so the class is dependency-free;
//   * the struct-serialization include and the runtime constraint checks that threw
//     std::invalid_argument are dropped (the caller validates constraints);
//   * everything else -- the profile equations, the sign selection with its anti-chattering
//     tie-break, the over-speed recovery, and the sampling order -- is kept as in the original,
//     equation references included, so this file can be diffed against upstream.
//
// This is NOT project code: keep it in src/third_party/, keep this header, and do not restyle it.
// ============================================================================================

#pragma once

#include <algorithm>
#include <cmath>

namespace wpilib {

// A trapezoid-shaped velocity profile.
//
// While this class can be used for a profiled movement from start to finish, the intended usage
// is to filter a reference's dynamics based on trapezoidal velocity constraints. To compute the
// reference obeying this constraint, do the following.
//
// Initialization:
//   TrapezoidProfile profile{{kMaxVelocity, kMaxAcceleration}};
//   State previousProfiledReference = initialReference;
//
// Run on update:
//   previousProfiledReference = profile.Calculate(timeSincePreviousUpdate,
//                                                 previousProfiledReference,
//                                                 unprofiledReference);
//
// where `unprofiledReference` is the reference at the current time and the "previous" reference
// is the profiled one from the previous update. Once the unprofiled reference is within the
// constraints, Calculate() returns the unprofiled reference unchanged.
class TrapezoidProfile {
   public:
    // Profile constraints.
    struct Constraints {
        double maxVelocity = 0.0;      // > 0
        double maxAcceleration = 0.0;  // > 0
    };

    // Profile state.
    struct State {
        double position = 0.0;
        double velocity = 0.0;

        constexpr bool operator==(const State& rhs) const {
            return position == rhs.position && velocity == rhs.velocity;
        }
    };

    // Profile timing.
    struct ProfileTiming {
        double t_1 = 0.0;  // time the profile spends accelerating
        double t_2 = 0.0;  // time the profile spends at the velocity limit
        double t_3 = 0.0;  // time the profile spends decelerating
    };

    constexpr explicit TrapezoidProfile(Constraints constraints) : m_constraints(constraints) {}

    // Calculates the position and velocity for the profile at a time t where the current state is
    // at time t = 0.
    //   t       The amount of time to advance from the current state toward the desired state.
    //   current The current state.
    //   goal    The desired state when the profile is complete.
    // Returns the position and velocity of the profile at time t.
    constexpr State Calculate(double t, State current, State goal) {
        // Sampled trajectory should start at the current state, regardless of validity.
        State sample{current};

        // Adjust states so that they are within the constraints and get the time required for the
        // current state to return to a valid state.
        const double recoveryTime = AdjustStates(current, goal);
        const double sign = GetSign(current, goal);
        m_profile = GenerateProfile(sign, current, goal);

        // In the case that the sign of the profile and the sign of the acceleration are identical,
        // the recovery can be treated as an extension of the first segment. In the case that they
        // differ, the recovered state will have a velocity of v_l and the above calculated t_1 will
        // be zero. To handle this, a check is added to the sampling of the first segment to ensure
        // proper recovery.
        m_profile.t_1 += recoveryTime;

        const auto advance = [](double time, double acceleration, State& state) {
            // x = x_i + v_i t + at^2 / 2   (2)
            state.position += state.velocity * time + acceleration / 2.0 * time * time;
            // v = v_i + at   (1)
            state.velocity += acceleration * time;
        };

        const double acceleration = sign * m_constraints.maxAcceleration;
        advance(std::min(t, m_profile.t_1),
                // Handle recovery to a feasible state if necessary.
                (recoveryTime > 0.0 && sample.velocity * sign > 0.0) ? -acceleration : acceleration,
                sample);

        if (t > m_profile.t_1) {
            t -= m_profile.t_1;
            advance(std::min(t, m_profile.t_2), 0.0, sample);

            if (t > m_profile.t_2) {
                t -= m_profile.t_2;
                advance(std::min(t, m_profile.t_3), -acceleration, sample);

                if (t > m_profile.t_3) {
                    sample = goal;
                }
            }
        }

        return sample;
    }

    // Returns the time to get between two states. This does not affect the internal variables,
    // and as a result, may be used for states not on the active trajectory.
    constexpr double TimeLeftUntil(State current, State goal) const {
        const double recoveryTime = AdjustStates(current, goal);
        const double sign = GetSign(current, goal);
        ProfileTiming profile = GenerateProfile(sign, current, goal);
        profile.t_1 += recoveryTime;
        return profile.t_1 + profile.t_2 + profile.t_3;
    }

    // Returns the duration of the profile, or zero if no goal was set.
    constexpr double Duration() const {
        return m_profile.t_1 + m_profile.t_2 + m_profile.t_3;
    }

    // Returns true if the profile has reached the goal: the time since the profile started has
    // exceeded the profile's total time.
    constexpr bool IsFinished(double t) const {
        return t >= Duration();
    }

   private:
    // Adjusts the profile states to be within the constraints and returns the time needed to
    // bring the current state back within the constraints.
    //
    // In order to smoothly return to a state within the constraints, the current state is
    // modified to be the result of accelerating towards a valid velocity at the maximum
    // acceleration. This method returns the time this transition takes. By contrast, the goal
    // velocity is simply clamped to the valid region.
    constexpr double AdjustStates(State& current, State& goal) const {
        if (std::abs(goal.velocity) > m_constraints.maxVelocity) {
            goal.velocity = std::copysign(m_constraints.maxVelocity, goal.velocity);
        }
        double recoveryTime = 0.0;
        const double violationAmount = std::abs(current.velocity) - m_constraints.maxVelocity;
        if (violationAmount > 0.0) {
            recoveryTime = violationAmount / m_constraints.maxAcceleration;
            // x = x_i + v_i t + at^2 / 2   (2)
            current.position +=
                current.velocity * recoveryTime + std::copysign(m_constraints.maxAcceleration, -current.velocity) * recoveryTime * recoveryTime / 2.0;
            // The closest valid velocity will have the magnitude of the max velocity.
            current.velocity = std::copysign(m_constraints.maxVelocity, current.velocity);
        }
        return recoveryTime;
    }

    // Returns the sign of the profile. The current and goal states must be within the profile
    // constraints for a valid sign. 1.0 if the profile direction is positive, -1.0 if it is not.
    constexpr double GetSign(const State& current, const State& goal) const {
        const double dx = goal.position - current.position;
        // Calculate threshold displacement
        // d = |v_t - v_i|(v_t + v_i) / (2 a_m)   (9)
        const double d = std::abs(goal.velocity - current.velocity) * (goal.velocity + current.velocity) / (2.0 * m_constraints.maxAcceleration);
        // As discussed in TrapezoidProfile.md, the correct sign must be chosen when dx == d because
        // following a suboptimal profile may lead to "chattering". Additionally, if numerical
        // precision errors cause the calculated optimal sign to change throughout the profile, that
        // may lead to suboptimal states being calculated. To fix this, we add a tolerance such that
        // if |dx - d| < epsilon, we return the sign that would lead to the minimum profile being
        // calculated. We do not have control over the floating point precision error from previous
        // calculations, and as such, it is difficult to bound the possible error. 1e-12 should be
        // good enough for FRC though.
        if (std::abs(dx - d) < 1e-12) {
            return std::copysign(1.0, goal.velocity);
        }
        return dx > d ? 1.0 : -1.0;
    }

    // Generates profile timings from valid current and goal states: the time for each section of
    // the profile from current and goal states with valid velocities.
    constexpr ProfileTiming GenerateProfile(double sign, const State& current, const State& goal) const {
        ProfileTiming profile{};
        const double acceleration = sign * m_constraints.maxAcceleration;
        const double velocityLimit = sign * m_constraints.maxVelocity;
        const double dx = goal.position - current.position;
        // Calculate the peak velocity to compare to velocity constraint.
        // v_p = sqrt(a dx + (v_t^2 + v_i^2) / 2)   (8)
        const double peakVelocity =
            sign * std::sqrt(std::max(acceleration * dx + (goal.velocity * goal.velocity + current.velocity * current.velocity) / 2.0, 0.0));
        // Handle the case where we hit maximum velocity.
        if (sign * peakVelocity > m_constraints.maxVelocity) {
            // t_1 = (v_l - v_i) / a   (13)
            profile.t_1 = (velocityLimit - current.velocity) / acceleration;
            // t_3 = (v_l - v_t) / a   (15)
            profile.t_3 = (velocityLimit - goal.velocity) / acceleration;
            // x_1 = (v_p^2 - v_i^2) / (2a)   (6), with v_l for v_p in the velocity-constrained case
            const double x_1 = (velocityLimit * velocityLimit - current.velocity * current.velocity) / (2.0 * acceleration);
            // x_3 = (v_p^2 - v_t^2) / (2a)   (7), with v_l for v_p in the velocity-constrained case
            const double x_3 = (velocityLimit * velocityLimit - goal.velocity * goal.velocity) / (2.0 * acceleration);
            // x_2 = dx - x_1 - x_3   (12)
            const double x_2 = dx - x_1 - x_3;
            // t_2 = x_2 / v_l   (14)
            profile.t_2 = x_2 / velocityLimit;
        } else {
            // t_1 = (v_p - v_i) / a   (13)
            profile.t_1 = (peakVelocity - current.velocity) / acceleration;
            // t_3 = (v_p - v_t) / a   (15)
            profile.t_3 = (peakVelocity - goal.velocity) / acceleration;
        }
        return profile;
    }

    Constraints m_constraints;
    ProfileTiming m_profile{};
};

}  // namespace wpilib
