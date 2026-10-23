#pragma once

// viam:ethercat:gantry -- one linear axis driven by a viam:ethercat servo motor.
//
// The gantry owns millimetres: the travel length, the conversion to motor revolutions, and the
// bound that keeps a move inside the axis. Everything below that (the cycle loop, the CiA402 mode
// switch, the homing procedure) belongs to the motor, which this component drives through the
// standard motor API plus the module's own do_command contract. It therefore depends on a motor
// component and never touches the EtherCAT bus itself.

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <viam/sdk/common/kinematics.hpp>
#include <viam/sdk/components/gantry.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/registry/registry.hpp>
#include <viam/sdk/resource/resource.hpp>

namespace ethercat::servo {

using namespace viam::sdk;

class GantryAxis final : public Gantry {
   public:
    static Model model();
    static std::shared_ptr<ModelRegistration> create_model_registration();
    // Validator: parses the attributes and returns the motor as this axis's dependency.
    static std::vector<std::string> validate(const ResourceConfig& cfg);

    GantryAxis(const Dependencies& deps, const ResourceConfig& cfg);

    GantryAxis(const GantryAxis&) = delete;
    GantryAxis& operator=(const GantryAxis&) = delete;
    GantryAxis(GantryAxis&&) = delete;
    GantryAxis& operator=(GantryAxis&&) = delete;
    ~GantryAxis() override = default;

    // --- rdk:component:gantry (one axis, so every vector has one entry) ---
    std::vector<double> get_position(const ProtoStruct& extra) override;
    void move_to_position(const std::vector<movement_coordinate>& coordinates, const ProtoStruct& extra) override;
    bool home(const ProtoStruct& extra) override;
    std::vector<double> get_lengths(const ProtoStruct& extra) override;
    bool is_moving() override;
    ProtoStruct get_status() override;
    void stop(const ProtoStruct& extra) override;
    ProtoStruct do_command(const ProtoStruct& command) override;
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& extra) override;
    KinematicsData get_kinematics(const ProtoStruct& extra) override;

   private:
    // Attributes. The homing block is passed straight to the motor's `home` verb, converted to
    // motor units, so the axis tuning lives in one place.
    struct Attributes {
        std::string motor;
        double length_mm = 0.0;
        double mm_per_rev = 0.0;
        double mm_per_sec = 0.0;  // default move speed; 0 = a speed is required per move
        bool home_negative = true;
        double home_mm_per_sec = 0.0;
        double home_accel_mm_per_sec2 = 0.0;
        double home_torque_percent = 0.0;
        double home_offset_mm = 0.0;
        double home_timeout_s = 120.0;
    };
    static Attributes parse(const ResourceConfig& cfg);
    // Resolve the attributes and the motor dependency. Called from the constructor: the SDK has no
    // reconfigure hook, so a config change builds a new component.
    void configure(const Dependencies& deps, const ResourceConfig& cfg);

    // Homing at the negative stop makes travel positive in motor revolutions; at the positive stop
    // the axis runs the other way and the sign mirrors.
    double sign() const noexcept {
        return attrs_.home_negative ? 1.0 : -1.0;
    }
    double mm_to_revs(double mm) const noexcept {
        return sign() * mm / attrs_.mm_per_rev;
    }
    double revs_to_mm(double revs) const noexcept {
        return sign() * revs * attrs_.mm_per_rev;
    }
    // The motor's `status`, which also proves the dependency is one of ours.
    ProtoStruct motor_status() const;
    // True once a homing run has succeeded this session, or when the configured absolute-encoder
    // object says the drive kept its reference across the restart.
    bool homed();
    bool absolute_reference();  // asks the motor once, then caches a positive answer

    Attributes attrs_;
    std::shared_ptr<Motor> motor_;
    bool abs_reference_ = false;  // cached positive result of the absolute-encoder check
};

}  // namespace ethercat::servo
