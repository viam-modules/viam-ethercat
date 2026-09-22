#include "viam/lib/motion_mode.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <string>

#include "ethercat/errors.hpp"
#include "ethercat/field.hpp"

namespace ethercat::servo {

namespace {

// Select the mode on the drive by SDO (SAFE-OP, pre-spawn). A drive that does not implement the
// configured mode refuses the value here, which fails start() with the reason.
void select_mode_on_drive(ConfigContext& cfg, Cia402Mode mode, const char* control_mode) {
    constexpr std::uint16_t kModesOfOp = 0x6060;
    const std::array<std::byte, 1> v{static_cast<std::byte>(static_cast<std::uint8_t>(mode))};
    try {
        cfg.sdo_write(kModesOfOp, 0, v);
    } catch (const SdoError& e) {
        throw Error(std::string("the drive refused control_mode '") + control_mode + "' (0x6060 <- " +
                    std::to_string(static_cast<int>(mode)) + "): " + e.what());
    }
}

}  // namespace

const char* to_string(MotionFault fault) noexcept {
    switch (fault) {
        case MotionFault::None:
            return "none";
        case MotionFault::SetpointNotAcknowledged:
            return "the drive did not acknowledge the new set-point (statusword bit 12)";
        case MotionFault::ModeNotAdopted:
            return "the drive did not adopt the commanded mode (0x6061 never echoed 0x6060)";
    }
    return "?";
}

// ============================================================================================
// ProfilePositionMode
// ============================================================================================

std::uint32_t ProfilePositionMode::resolve(ConfigContext& cfg) {
    select_mode_on_drive(cfg, Cia402Mode::ProfilePosition, "profile");
    // Resolve the PDO fields (controlword, statusword, 0x6060/0x6061, targets) and, when a quick-stop
    // deceleration is configured, run the one-time quick-stop SDOs: assert 0x605A, write and read back
    // 0x6085 (throws on a mismatch). Returns the echoed 0x6085 for the controller's velocity budget.
    return sequencer_.configure(cfg, /*needs_quick_stop=*/quick_stop_decel_ > 0);
}

void ProfilePositionMode::reset() noexcept {
    sequencer_.reset();
    intent_ = Cia402Mode::ProfilePosition;
    target_counts_ = 0;
    profile_vel_ = 0;
    pv_velocity_ = 0;
    halted_ = false;
    pending_new_setpoint_ = false;
    echo_wait_cycles_ = 0;
    mode_not_adopted_ = false;
}

void ProfilePositionMode::go_to(std::int32_t target_counts, std::uint32_t velocity_cps) noexcept {
    intent_ = Cia402Mode::ProfilePosition;
    target_counts_ = target_counts;
    profile_vel_ = velocity_cps;
    pending_new_setpoint_ = true;  // arm the PP handshake once the drive echoes PP
    halted_ = false;
}

void ProfilePositionMode::set_velocity(std::int32_t velocity_cps) noexcept {
    intent_ = Cia402Mode::ProfileVelocity;
    pv_velocity_ = velocity_cps;
    halted_ = false;
}

std::uint16_t ProfilePositionMode::step(CycleContext& ctx, const MotionFeedback& fb) noexcept {
    mode_not_adopted_ = false;
    SequencerCommand cmd;
    cmd.mode = intent_;  // the sequencer writes 0x6060 = cmd.mode every cycle
    cmd.enable = true;
    cmd.halt = halted_;
    cmd.target_counts = target_counts_;
    cmd.profile_velocity = profile_vel_;
    cmd.target_velocity = pv_velocity_;

    // Until 0x6061 echoes the commanded mode, hold at rest (no set-point, zero velocity) and count.
    const bool echoed = !fb.mode_echo || *fb.mode_echo == static_cast<std::int8_t>(intent_);
    if (!echoed) {
        if (++echo_wait_cycles_ == kModeEchoCycles) {
            // Reported once (the controller fails the command), then fall back to the mode the drive
            // actually runs with no pending command, so is_moving() and the hold stay consistent.
            mode_not_adopted_ = true;
            intent_ = static_cast<Cia402Mode>(*fb.mode_echo);
            pv_velocity_ = 0;
            pending_new_setpoint_ = false;
        }
        cmd.new_setpoint = false;
        cmd.target_velocity = 0;
        return sequencer_.step(ctx, cmd);
    }
    echo_wait_cycles_ = 0;
    cmd.new_setpoint = pending_new_setpoint_;
    pending_new_setpoint_ = false;
    return sequencer_.step(ctx, cmd);
}

std::uint16_t ProfilePositionMode::step_shutdown(CycleContext& ctx, const MotionFeedback& fb) noexcept {
    (void)fb;
    // With a quick-stop deceleration configured the sequencer runs the drive's quick-stop (0x6085, then
    // auto SwitchOnDisabled); without one (the default, 0) the stop is a disable-voltage coast.
    if (quick_stop_decel_ > 0) {
        SequencerCommand scmd;
        scmd.mode = commanded_mode();
        scmd.enable = false;  // the sequencer's shutdown branch owns the controlword
        return sequencer_.step(ctx, scmd);
    }
    return ControlWord::disable_voltage();
}

bool ProfilePositionMode::is_target_reached(const MotionFeedback& fb) const noexcept {
    if (intent_ != Cia402Mode::ProfilePosition || !sequencer_.state().handshake_idle) {
        return false;
    }
    // Statusword bit 10 when the drive implements it; else within tolerance and at rest.
    return fb.target_reached_bit_usable ? fb.status.target_reached() : (fb.near_target && fb.at_rest);
}

MotionFault ProfilePositionMode::fault() const noexcept {
    if (mode_not_adopted_) {
        return MotionFault::ModeNotAdopted;
    }
    return sequencer_.state().handshake_timed_out ? MotionFault::SetpointNotAcknowledged : MotionFault::None;
}

// ============================================================================================
// CyclicPositionMode
// ============================================================================================

std::uint32_t CyclicPositionMode::resolve(ConfigContext& cfg) {
    select_mode_on_drive(cfg, Cia402Mode::CyclicSyncPosition, "csp");
    f_target_pos_ = cfg.resolve_rx<cia402::TargetPosition>();  // the one command object CSP streams
    return 0;                                                  // no drive-side quick-stop: the generator owns the stop
}

void CyclicPositionMode::reset() noexcept {
    generator_.reseed(0);
    positioning_ = true;
}

std::uint32_t CyclicPositionMode::shutdown_cycles() const noexcept {
    // Ramp from the velocity ceiling to rest at accel_, plus a margin, so the shutdown window
    // outlasts the ramp and torque is never cut at speed.
    const double ramp_s = accel_ > 0.0 ? max_vel_cps_ / accel_ : 0.0;
    const double cycles = (ramp_s / dt_s_) + 20.0;
    return static_cast<std::uint32_t>(std::min(cycles, 60000.0));
}

void CyclicPositionMode::go_to(std::int32_t target_counts, std::uint32_t velocity_cps) noexcept {
    // The controller rejects a non-positive speed upstream; a refused set_goal leaves the generator holding.
    if (generator_.set_goal(target_counts, static_cast<double>(velocity_cps), accel_)) {
        positioning_ = true;
    }
}

void CyclicPositionMode::set_velocity(std::int32_t velocity_cps) noexcept {
    if (generator_.set_velocity(static_cast<double>(velocity_cps), accel_)) {
        positioning_ = false;
    }
}

std::uint16_t CyclicPositionMode::step(CycleContext& ctx, const MotionFeedback& fb) noexcept {
    (void)fb;
    ctx.store<cia402::TargetPosition::type>(f_target_pos_, generator_.step(dt_s_));
    return ControlWord::enable_operation();  // CSP has no handshake bits; the target does the work
}

void CyclicPositionMode::track(CycleContext& ctx, const MotionFeedback& fb) noexcept {
    // Not enabled: the target follows the shaft so the first enabled cycle commands "stay here"
    // (a CSP drive enabled against a stale 0x607A lunges to it).
    generator_.reseed(fb.actual);
    ctx.store<cia402::TargetPosition::type>(f_target_pos_, fb.actual);
}

std::uint16_t CyclicPositionMode::step_shutdown(CycleContext& ctx, const MotionFeedback& fb) noexcept {
    // Ramp to rest while energized, then disable voltage. A drive that is already off goes straight to disable.
    if (!fb.status.operation_enabled() || fb.status.fault()) {
        generator_.reseed(fb.actual);
        ctx.store<cia402::TargetPosition::type>(f_target_pos_, fb.actual);
        return ControlWord::disable_voltage();
    }
    if (!generator_.idle()) {
        if (generator_.phase() != TrapezoidGenerator::Phase::Velocity || generator_.velocity() != 0.0) {
            generator_.stop(accel_);
        }
        ctx.store<cia402::TargetPosition::type>(f_target_pos_, generator_.step(dt_s_));
        return ControlWord::enable_operation();
    }
    ctx.store<cia402::TargetPosition::type>(f_target_pos_, generator_.position());
    return ControlWord::disable_voltage();
}

}  // namespace ethercat::servo
