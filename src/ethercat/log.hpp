#pragma once

// ethercat::log -- the library's diagnostic log for cold paths (open/configure/bring-up).
// One process-wide level and one sink. The default sink prints to stderr; a consumer installs its
// own (the Viam module forwards to the SDK's resource logger). ETHERCAT_LOG checks the level before
// formatting, so a disabled level costs one relaxed atomic load. The RT loop may emit only one-shot
// lines on a phase transition, never per cycle.

#include <atomic>
#include <cstdint>
#include <format>
#include <memory>
#include <span>
#include <string>
#include <string_view>

namespace ethercat::log {

enum class Level : std::uint8_t {
    Error = 0,
    Warn = 1,
    Info = 2,
    Debug = 3,
};

const char* to_string(Level level) noexcept;

void set_level(Level level) noexcept;
Level level() noexcept;
inline bool enabled(Level at) noexcept {
    return static_cast<std::uint8_t>(at) <= static_cast<std::uint8_t>(level());
}

// Where a line was emitted (the macros fill it in), for sinks that show source locations.
struct Origin {
    const char* file = "";
    unsigned line = 0;
};

// Receives every line that passes the level filter. `tag` names the source ("sdo", "bringup", ...).
class Sink {
   public:
    virtual ~Sink() = default;
    virtual void write(Level level, std::string_view tag, std::string_view message, Origin origin) noexcept = 0;
};

void set_sink(std::shared_ptr<Sink> sink) noexcept;
// Restore stderr if `sink` is the installed one (an owner going away).
void clear_sink(const Sink* sink) noexcept;

// Emit one line; no-op when `at` is above the current level. When no sink are configured default to stderr
void write(Level at, std::string_view tag, std::string_view msg, Origin origin = {}) noexcept;

// "01 02 0A FF" for a byte span.
std::string hex_bytes(std::span<const std::byte> bytes);

}  // namespace ethercat::log

// Format and emit, gated on the level first.
#define ETHERCAT_LOG(level_, tag_, ...)                                                                                    \
    do {                                                                                                                   \
        if (::ethercat::log::enabled(level_)) {                                                                            \
            ::ethercat::log::write(level_, tag_, ::std::format(__VA_ARGS__), ::ethercat::log::Origin{__FILE__, __LINE__}); \
        }                                                                                                                  \
    } while (false)

#define ETHERCAT_LOG_DEBUG(tag_, ...) ETHERCAT_LOG(::ethercat::log::Level::Debug, tag_, __VA_ARGS__)
#define ETHERCAT_LOG_INFO(tag_, ...) ETHERCAT_LOG(::ethercat::log::Level::Info, tag_, __VA_ARGS__)
#define ETHERCAT_LOG_WARN(tag_, ...) ETHERCAT_LOG(::ethercat::log::Level::Warn, tag_, __VA_ARGS__)
#define ETHERCAT_LOG_ERROR(tag_, ...) ETHERCAT_LOG(::ethercat::log::Level::Error, tag_, __VA_ARGS__)
