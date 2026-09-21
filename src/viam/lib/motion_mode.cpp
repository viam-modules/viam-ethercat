#include "viam/lib/motion_mode.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "ethercat/errors.hpp"
#include "ethercat/field.hpp"

namespace ethercat::servo {

namespace {

// Cycles to wait for 0x6061 to echo a newly commanded 0x6060 before the change counts as failed.
// Drives adopt a PDO-carried mode within a few cycles; this bound (200 ms at 1 kHz) only turns a drive
// that never follows into an error instead of an indefinite hold.
constexpr std::uint32_t kModeEchoCycles = 200;

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
            mode_not_adopted_ = true;  // reported once; the controller fails the command
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

}  // namespace ethercat::servo
