#pragma once

// MotionMode: how one CiA402 mode family turns the motor API into process data. The controller
// owns everything mode-independent (bus, enable ladder, faults, command queue, published state)
// and calls one of these every cycle. RT contract: all but resolve()/reset() are noexcept,
// allocation-free and RT-thread only; resolve() runs pre-spawn and may throw (fails start()).

#include <cstdint>
#include <memory>
#include <optional>

#include "ethercat/cia402.hpp"
#include "ethercat/cia402_sequencer.hpp"
#include "ethercat/master.hpp"
#include "ethercat/runner.hpp"
#include "viam/lib/servo_config.hpp"
#include "viam/lib/trapezoid.hpp"

namespace ethercat::servo {

// Cycles to wait for 0x6061 to echo a commanded 0x6060 before treating the drive as not following.
// Drives adopt a mode within a few cycles; this bound (200 ms at 1 kHz) only turns a drive that never
// follows into an error instead of an indefinite hold.
constexpr std::uint32_t kModeEchoCycles = 200;

// One cycle's inputs, read by the controller from the latched image.
struct MotionFeedback {
    Status status{};
    std::int32_t actual = 0;                // 0x6064
    std::optional<std::int8_t> mode_echo;   // 0x6061, when mapped
    bool at_rest = false;                   // position-stability verdict (one cycle stale)
    bool near_target = false;               // |actual - target()| <= position tolerance
    bool target_reached_bit_usable = true;  // false on drives that tie statusword bit 10 high
};

enum class MotionCommand : std::uint8_t { Position, Velocity };

// A failure the mode detected this cycle. The controller fails the command and logs it.
enum class MotionFault : std::uint8_t { None, SetpointNotAcknowledged, ModeNotAdopted };

const char* to_string(MotionFault fault) noexcept;

class MotionMode {
   public:
    MotionMode() = default;
    MotionMode(const MotionMode&) = delete;
    MotionMode& operator=(const MotionMode&) = delete;
    MotionMode(MotionMode&&) = delete;
    MotionMode& operator=(MotionMode&&) = delete;
    virtual ~MotionMode() = default;

    virtual const char* name() const noexcept = 0;           // "PP", "CSP"
    virtual Cia402Mode commanded_mode() const noexcept = 0;  // the 0x6060 value this cycle
    // The 0x6060 value a command needs. A command that needs a different mode than the drive is
    // running is refused while the motor moves; a mode change is only requested at rest.
    virtual Cia402Mode mode_required_by(MotionCommand cmd) const noexcept = 0;

    // Non-RT, pre-spawn: resolve the PDO fields and run one-time SDOs. Returns the echoed 0x6085, else 0.
    virtual std::uint32_t resolve(ConfigContext& cfg) = 0;
    virtual void reset() noexcept = 0;  // per-run RT state
    // Cycles this mode needs to ramp a moving axis to rest at shutdown (0: the drive's quick-stop
    // does it). The controller sizes the shutdown window with it so torque is never cut at speed.
    virtual std::uint32_t shutdown_cycles() const noexcept = 0;

    virtual void go_to(std::int32_t target_counts, std::uint32_t velocity_cps) noexcept = 0;
    virtual void go_for(std::int32_t delta_counts, std::uint32_t velocity_cps, std::int32_t actual) noexcept {
        go_to(static_cast<std::int32_t>(static_cast<std::uint32_t>(actual) + static_cast<std::uint32_t>(delta_counts)), velocity_cps);
    }
    virtual void set_velocity(std::int32_t velocity_cps) noexcept = 0;
    virtual void halt() noexcept = 0;  // ramp to rest and hold, energized, until the next go_to/set_velocity

    // OperationEnabled: write the command objects, return the controlword.
    virtual std::uint16_t step(CycleContext& ctx, const MotionFeedback& fb) noexcept = 0;
    // Not enabled (bring-up, Init, Faulted, ...): keep the command objects consistent with the actual position.
    virtual void track(CycleContext& ctx, const MotionFeedback& fb) noexcept = 0;
    // Module shutdown: ramp to rest, then return disable-voltage.
    virtual std::uint16_t step_shutdown(CycleContext& ctx, const MotionFeedback& fb) noexcept = 0;

    virtual std::int32_t target() const noexcept = 0;
    virtual bool is_target_reached(const MotionFeedback& fb) const noexcept = 0;  // the current positioning move is complete
    virtual bool is_moving() const noexcept = 0;                                  // commanding motion (a velocity run or a ramp)
    virtual MotionFault fault() const noexcept {
        return MotionFault::None;
    }
};

// PP for go_to/go_for, PV for set_rpm; the drive generates the trajectory (handshake in
// Cia402Sequencer). 0x6060 follows the last command and the motion body waits for the 0x6061 echo.
class ProfilePositionMode final : public MotionMode {
   public:
    ProfilePositionMode(std::uint32_t quick_stop_decel, std::int16_t quick_stop_option) noexcept
        : quick_stop_decel_(quick_stop_decel), sequencer_(quick_stop_decel, quick_stop_option) {}

    const char* name() const noexcept override {
        return "PP";
    }
    Cia402Mode commanded_mode() const noexcept override {
        return intent_;
    }
    Cia402Mode mode_required_by(MotionCommand cmd) const noexcept override {
        return cmd == MotionCommand::Position ? Cia402Mode::ProfilePosition : Cia402Mode::ProfileVelocity;
    }
    std::uint32_t resolve(ConfigContext& cfg) override;
    void reset() noexcept override;
    std::uint32_t shutdown_cycles() const noexcept override {
        return 0;  // the drive's quick-stop ramp (0x6085) does it
    }

    void go_to(std::int32_t target_counts, std::uint32_t velocity_cps) noexcept override;
    void set_velocity(std::int32_t velocity_cps) noexcept override;
    void halt() noexcept override {
        halted_ = true;  // CiA402 Halt (bit 8)
    }

    std::uint16_t step(CycleContext& ctx, const MotionFeedback& fb) noexcept override;
    void track(CycleContext& ctx, const MotionFeedback& fb) noexcept override {
        (void)ctx;
        (void)fb;  // the drive holds its own set-points
    }
    std::uint16_t step_shutdown(CycleContext& ctx, const MotionFeedback& fb) noexcept override;

    std::int32_t target() const noexcept override {
        return target_counts_;
    }
    bool is_target_reached(const MotionFeedback& fb) const noexcept override;
    bool is_moving() const noexcept override {
        return intent_ == Cia402Mode::ProfileVelocity && pv_velocity_ != 0 &&
               !halted_;  // a PP move is tracked by the controller's move generation
    }
    MotionFault fault() const noexcept override;

   private:
    const std::uint32_t quick_stop_decel_;
    Cia402Sequencer sequencer_;

    // RT-only working state
    Cia402Mode intent_ = Cia402Mode::ProfilePosition;  // the mode of the last command
    std::int32_t target_counts_ = 0;
    std::uint32_t profile_vel_ = 0;
    std::int32_t pv_velocity_ = 0;
    bool halted_ = false;
    bool pending_new_setpoint_ = false;
    std::uint32_t echo_wait_cycles_ = 0;  // cycles 0x6061 has lagged intent_
    bool mode_not_adopted_ = false;       // one cycle, when the echo wait expires
};

// Cyclic Synchronous Position: the master streams 0x607A every cycle from a TrapezoidGenerator;
// go_to/go_for are goals, set_rpm a velocity run, halt a ramp to rest. The drive only closes its
// position loop. For drives that implement only the cyclic synchronous modes.
class CyclicPositionMode final : public MotionMode {
   public:
    // dt_s: cycle period; max_accel_cps2: ramp acceleration; max_vel_cps: the ceiling the shutdown ramp starts from.
    CyclicPositionMode(double dt_s, double max_accel_cps2, double max_vel_cps) noexcept
        : dt_s_(dt_s), accel_(max_accel_cps2), max_vel_cps_(max_vel_cps) {}

    const char* name() const noexcept override {
        return "CSP";
    }
    Cia402Mode commanded_mode() const noexcept override {
        return Cia402Mode::CyclicSyncPosition;
    }
    Cia402Mode mode_required_by(MotionCommand cmd) const noexcept override {
        (void)cmd;
        return Cia402Mode::CyclicSyncPosition;  // one mode for both kinds of command
    }
    std::uint32_t resolve(ConfigContext& cfg) override;
    void reset() noexcept override;
    std::uint32_t shutdown_cycles() const noexcept override;

    void go_to(std::int32_t target_counts, std::uint32_t velocity_cps) noexcept override;
    void set_velocity(std::int32_t velocity_cps) noexcept override;
    void halt() noexcept override {
        generator_.stop(accel_);  // ramp to rest, then hold the position reached
    }

    std::uint16_t step(CycleContext& ctx, const MotionFeedback& fb) noexcept override;
    void track(CycleContext& ctx, const MotionFeedback& fb) noexcept override;
    std::uint16_t step_shutdown(CycleContext& ctx, const MotionFeedback& fb) noexcept override;

    std::int32_t target() const noexcept override {
        return generator_.goal();
    }
    bool is_target_reached(const MotionFeedback& fb) const noexcept override {
        return positioning_ && generator_.idle() && fb.near_target && fb.at_rest;  // bit 10 is a status toggle in CSP
    }
    bool is_moving() const noexcept override {
        return !generator_.idle();  // the streamed target is changing
    }

   private:
    const double dt_s_;
    const double accel_;
    const double max_vel_cps_;
    FieldLocation f_target_pos_{};  // 0x607A (required)
    TrapezoidGenerator generator_;
    bool positioning_ = true;  // last command: a positioning move (true) or a velocity run (false)
};

}  // namespace ethercat::servo
