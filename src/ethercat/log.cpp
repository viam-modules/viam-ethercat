#include "ethercat/log.hpp"

#include <atomic>
#include <cstdio>
#include <memory>
#include <string>

namespace ethercat::log {

namespace {

std::atomic<Level> g_level{Level::Info};
std::atomic<std::shared_ptr<Sink>> g_sink{nullptr};  // nullptr = stderr

}  // namespace

const char* to_string(Level level) noexcept {
    switch (level) {
        case Level::Error:
            return "ERROR";
        case Level::Warn:
            return "WARN";
        case Level::Info:
            return "INFO";
        case Level::Debug:
            return "DEBUG";
    }
    return "?";
}

void set_level(Level level) noexcept {
    g_level.store(level, std::memory_order_relaxed);
}

Level level() noexcept {
    return g_level.load(std::memory_order_relaxed);
}

void set_sink(std::shared_ptr<Sink> sink) noexcept {
    g_sink.store(std::move(sink), std::memory_order_release);
}

void clear_sink(const Sink* sink) noexcept {
    std::shared_ptr<Sink> cur = g_sink.load(std::memory_order_acquire);
    if (cur.get() == sink) {
        g_sink.compare_exchange_strong(cur, nullptr, std::memory_order_acq_rel);
    }
}

void write(Level at, std::string_view tag, std::string_view msg, Origin origin) noexcept {
    if (!enabled(at)) {
        return;
    }
    if (const std::shared_ptr<Sink> sink = g_sink.load(std::memory_order_acquire)) {
        sink->write(at, tag, msg, origin);
        return;
    }
    (void)std::fprintf(stderr,
                       "[ethercat] %s %.*s: %.*s\n",
                       to_string(at),
                       static_cast<int>(tag.size()),
                       tag.data(),
                       static_cast<int>(msg.size()),
                       msg.data());
    (void)std::fflush(stderr);
}

std::string hex_bytes(std::span<const std::byte> bytes) {
    static constexpr char kDigits[] = "0123456789ABCDEF";
    std::string s;
    s.reserve(bytes.size() * 3);
    for (const std::byte b : bytes) {
        if (!s.empty()) {
            s.push_back(' ');
        }
        const auto v = static_cast<unsigned>(b);
        s.push_back(kDigits[v >> 4U]);
        s.push_back(kDigits[v & 0x0FU]);
    }
    return s;
}

}  // namespace ethercat::log
