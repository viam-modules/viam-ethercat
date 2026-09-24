// ethercat::log -- level gating, the sink hook, and the stderr line format.

#include <array>
#include <cstddef>
#include <memory>
#include <string>
#include <string_view>

#include "ethercat/log.hpp"
#include "test_harness.hpp"

using ethercat::log::Level;

namespace {

// Records every line handed to it.
class Capture final : public ethercat::log::Sink {
   public:
    void write(Level level, std::string_view tag, std::string_view message, ethercat::log::Origin origin) noexcept override {
        last_origin = origin;
        lines += std::string(ethercat::log::to_string(level)) + " " + std::string(tag) + ": " + std::string(message) + "\n";
    }
    std::string lines;
    ethercat::log::Origin last_origin{};
};

template <class Fn>
std::string capture(Fn&& fn) {
    auto sink = std::make_shared<Capture>();
    ethercat::log::set_sink(sink);
    fn();
    ethercat::log::clear_sink(sink.get());
    return sink->lines;
}

}  // namespace

TEST("a line above the current level is suppressed; at or below reaches the sink") {
    ethercat::log::set_level(Level::Info);
    const std::string quiet = capture([] { ETHERCAT_LOG_DEBUG("sdo", "write {} bytes", 4); });
    CHECK(quiet.empty());

    const std::string loud = capture([] { ETHERCAT_LOG_INFO("configure", "slave {} remapped", 1); });
    CHECK_EQ(loud, std::string("INFO configure: slave 1 remapped\n"));

    ethercat::log::set_level(Level::Debug);
    const std::string dbg = capture([] { ETHERCAT_LOG_DEBUG("sdo", "write {} bytes", 4); });
    CHECK_EQ(dbg, std::string("DEBUG sdo: write 4 bytes\n"));
    ethercat::log::set_level(Level::Info);
}

TEST("the macro hands the emitting file and line to the sink") {
    auto sink = std::make_shared<Capture>();
    ethercat::log::set_sink(sink);
    ETHERCAT_LOG_INFO("t", "here");
    const unsigned line = __LINE__ - 1;
    ethercat::log::clear_sink(sink.get());
    CHECK_EQ(std::string(sink->last_origin.file), std::string(__FILE__));
    CHECK_EQ(sink->last_origin.line, line);
}

TEST("clear_sink only removes the sink it names") {
    auto a = std::make_shared<Capture>();
    auto b = std::make_shared<Capture>();
    ethercat::log::set_sink(a);
    ethercat::log::clear_sink(b.get());  // not installed: no effect
    ETHERCAT_LOG_INFO("t", "x");
    CHECK_EQ(a->lines, std::string("INFO t: x\n"));
    ethercat::log::clear_sink(a.get());
    ETHERCAT_LOG_INFO("t", "y");  // goes to stderr now
    CHECK_EQ(a->lines, std::string("INFO t: x\n"));
}

TEST("hex_bytes renders upper-case space-separated bytes") {
    const std::array<std::byte, 4> b{std::byte{0x01}, std::byte{0x00}, std::byte{0xAB}, std::byte{0xFF}};
    CHECK_EQ(ethercat::log::hex_bytes(b), std::string("01 00 AB FF"));
    CHECK_EQ(ethercat::log::hex_bytes(std::span<const std::byte>{}), std::string(""));
}

TEST_MAIN()
