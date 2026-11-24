// viam:ethercat:gantry without a drive: attribute parsing (and the kinematics file it reads on the
// way), plus the axis's behaviour against a fake motor where the drive does not matter.

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/components/motor.hpp>
#include <viam/sdk/config/resource.hpp>
#include <viam/sdk/resource/resource.hpp>
#include <viam/sdk/resource/resource_api.hpp>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/module/gantry_axis.hpp"

using ethercat::Error;
using ethercat::servo::GantryAxis;
using viam::sdk::API;
using viam::sdk::Dependencies;
using viam::sdk::GeometryConfig;
using viam::sdk::Motor;
using viam::sdk::Name;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;
using viam::sdk::ResourceConfig;

namespace {

const char* const kSva = R"({"name":"x-axis","kinematic_param_type":"SVA","links":[],"joints":[]})";

// A valid config, plus whatever `attrs` adds.
ResourceConfig config(ProtoStruct attrs = {}) {
    attrs.emplace("motor", ProtoValue(std::string("x-motor")));
    attrs.emplace("length_mm", ProtoValue(500.0));
    attrs.emplace("mm_per_rev", ProtoValue(40.0));
    return ResourceConfig("gantry", "x-axis", "rdk", std::move(attrs), "rdk:component:gantry", GantryAxis::model());
}

GantryAxis::Attributes parse_with_file(ProtoValue kinematics_file) {
    ProtoStruct a;
    a.emplace("kinematics_file", std::move(kinematics_file));
    return GantryAxis::parse(config(std::move(a)));
}

// Overwritten by every test that uses it, so a leftover from an earlier run is never read.
std::string temp_file(const std::string& contents) {
    const std::string path = (std::filesystem::temp_directory_path() / "gantry_axis_test.json").string();
    std::ofstream(path, std::ios::binary) << contents;
    return path;
}

// The motor as the gantry sees it over RPC: it passes the describe probe, and its stop() either
// works or fails the way a call to a motor the RDK has already removed does. With `travelling`
// set, go_to blocks, as on a drive mid-travel, until a stop arrives.
class FakeMotor final : public Motor {
   public:
    FakeMotor() : Motor("x-motor") {}

    int stops = 0;
    bool unreachable = false;
    bool travelling = false;
    // Set by stop() and read by the gantry's worker thread, which runs the blocked call.
    std::atomic<bool> stopped{false};
    // The blocked call ends normally on a stop, as a move does that arrived just before the halt.
    bool arrives_on_stop = false;
    // is_moving() answers true this many more times after a stop: the axis ramping down.
    int ramp_polls = 0;
    // The drive's frame, as the motor reports it: the axis's zero and where the shaft is now.
    double home_revolutions = 0.0;
    double position_revs = 0.0;
    double last_go_to_revs = 0.0;

    void stop(const ProtoStruct& /*extra*/) override {
        ++stops;
        if (unreachable) {
            throw std::runtime_error("Unavailable: motor is being removed");
        }
        stopped = true;
    }
    void travel() {
        while (travelling && !stopped) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    ProtoStruct do_command(const ProtoStruct& command) override {
        ProtoStruct reply;
        if (command.find("status") != command.end()) {
            ProtoStruct status;
            status.emplace("homed", ProtoValue(true));
            status.emplace("home_revolutions", ProtoValue(home_revolutions));
            reply.emplace("status", ProtoValue(std::move(status)));
            return reply;
        }
        ProtoStruct describe;
        describe.emplace("driver", ProtoValue(std::string("viam:ethercat")));
        describe.emplace("homing", ProtoValue(true));
        reply.emplace("describe", ProtoValue(std::move(describe)));
        return reply;
    }
    void set_power(double /*power_pct*/, const ProtoStruct& /*extra*/) override {}
    void go_for(double /*rpm*/, double /*revolutions*/, const ProtoStruct& /*extra*/) override {}
    void go_to(double /*rpm*/, double position_revolutions, const ProtoStruct& /*extra*/) override {
        last_go_to_revs = position_revolutions;
        travel();
        if (stopped && !arrives_on_stop) {
            throw std::runtime_error("move aborted (motor stopped)");
        }
    }
    void set_rpm(double /*rpm*/, const ProtoStruct& /*extra*/) override {}
    void reset_zero_position(double /*offset*/, const ProtoStruct& /*extra*/) override {}
    position get_position(const ProtoStruct& /*extra*/) override {
        return position_revs;
    }
    properties get_properties(const ProtoStruct& /*extra*/) override {
        return {};
    }
    power_status get_power_status(const ProtoStruct& /*extra*/) override {
        return {true, 1.0};
    }
    bool is_moving() override {
        if (stopped && ramp_polls > 0) {
            --ramp_polls;
            return true;
        }
        return false;
    }
    ProtoStruct get_status() override {
        return {};
    }
    std::vector<GeometryConfig> get_geometries(const ProtoStruct& /*extra*/) override {
        return {};
    }
};

Dependencies deps_with(const std::shared_ptr<FakeMotor>& motor) {
    Dependencies deps;
    deps.emplace(Name(API::from_string("rdk:component:motor"), "", "x-motor"), motor);
    return deps;
}

// A probe that reports the caller gone after a few polls, so the motor's call is under way first.
std::function<bool()> cancel_after_polls(int polls) {
    return [n = 0, polls]() mutable { return ++n > polls; };
}

}  // namespace

TEST("no kinematics_file: the axis has no kinematics") {
    const GantryAxis::Attributes attrs = GantryAxis::parse(config());
    CHECK_EQ(attrs.motor, std::string("x-motor"));
    CHECK(attrs.kinematics.empty());
}

TEST("kinematics_file: the file's bytes are kept verbatim") {
    const std::vector<unsigned char> k = parse_with_file(ProtoValue(temp_file(kSva))).kinematics;
    CHECK_EQ(std::string(k.begin(), k.end()), std::string(kSva));
}

TEST("kinematics_file: a missing or empty file is a config error, not a silent fixed frame") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string("/nonexistent/gantry_axis_test.json"))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(temp_file(""))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string())), Error, "missing or empty");
}

TEST("kinematics_file: must be a string") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(42.0)), Error, "wrong type");
}

TEST("validate names the motor as the only dependency") {
    const std::vector<std::string> deps = GantryAxis::validate(config());
    CHECK_EQ(deps.size(), std::size_t{1});
    CHECK_EQ(deps[0], std::string("x-motor"));
}

// The motor changes this frame without telling anyone -- a home through its own do_command, or a
// restart on an absolute encoder -- so an axis holding a copy commands millimetres in a dead frame.
TEST("millimetres follow the motor's current zero, not one captured at home()") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());

    motor->home_revolutions = 7.0;  // homed somewhere else, without the axis being asked
    motor->position_revs = 7.0;
    CHECK_EQ(axis.get_position({}).front(), 0.0);

    axis.move_to_position({{80.0, 10.0}}, {});  // 80 mm at 40 mm/rev: two revs past that zero
    CHECK_EQ(motor->last_go_to_revs, 9.0);
}

TEST("stop forwards to the motor") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    axis.stop({});
    CHECK_EQ(motor->stops, 1);
}

TEST("stop survives a motor that is already gone, so removal on shutdown cannot fail") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    motor->unreachable = true;
    axis.stop({});
    CHECK_EQ(motor->stops, 1);
}

TEST("a cancel whose stop fails returns an error within the timeout instead of hanging") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    axis.set_cancel_probe(cancel_after_polls(3), std::chrono::milliseconds(300));
    motor->travelling = true;
    motor->unreachable = true;
    const auto started = std::chrono::steady_clock::now();
    CHECK_THROWS_MSG(axis.move_to_position({{80.0, 10.0}}, {}), Error, "its stop failed (Unavailable: motor is being removed)");
    CHECK(std::chrono::steady_clock::now() - started < std::chrono::seconds(3));
    CHECK(motor->stops >= 1);
    motor->stopped = true;  // let the worker still blocked in the motor's call finish
}

TEST("a cancelled move stops the motor and returns only once the axis is at rest") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    axis.set_cancel_probe(cancel_after_polls(3), std::chrono::seconds(5));
    motor->travelling = true;
    motor->ramp_polls = 5;
    CHECK_THROWS_MSG(axis.move_to_position({{80.0, 10.0}}, {}), Error, "cancelled by the caller: the motor stopped, and the axis is at rest");
    CHECK(motor->stops >= 1);
    CHECK_EQ(motor->ramp_polls, 0);
}

TEST("a move that arrives as the cancel lands stays a completed move") {
    const auto motor = std::make_shared<FakeMotor>();
    GantryAxis axis(deps_with(motor), config());
    axis.set_cancel_probe(cancel_after_polls(3), std::chrono::seconds(5));
    motor->travelling = true;
    motor->arrives_on_stop = true;
    axis.move_to_position({{80.0, 10.0}}, {});
    CHECK_EQ(motor->last_go_to_revs, 2.0);
}

TEST_MAIN()
