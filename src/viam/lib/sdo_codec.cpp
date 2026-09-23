#include "viam/lib/sdo_codec.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

#include "ethercat/errors.hpp"

namespace ethercat::servo::sdo_codec {

namespace {

constexpr double kExactDouble = 9007199254740992.0;  // 2^53

std::string lower(std::string_view s) {
    std::string out;
    out.reserve(s.size());
    for (const char c : s) {
        out.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));
    }
    return out;
}

// Parse "0x1F", "-12" or "12" into a signed 128-bit-ish range via long double-free integer parsing.
bool parse_integer(std::string_view text, bool& negative, std::uint64_t& magnitude) {
    std::size_t i = 0;
    negative = false;
    if (i < text.size() && (text[i] == '-' || text[i] == '+')) {
        negative = text[i] == '-';
        ++i;
    }
    int base = 10;
    if (i + 1 < text.size() && text[i] == '0' && (text[i + 1] == 'x' || text[i + 1] == 'X')) {
        base = 16;
        i += 2;
    }
    if (i >= text.size()) {
        return false;
    }
    magnitude = 0;
    for (; i < text.size(); ++i) {
        const char c = text[i];
        int d = 0;
        if (c >= '0' && c <= '9') {
            d = c - '0';
        } else if (base == 16 && c >= 'a' && c <= 'f') {
            d = 10 + (c - 'a');
        } else if (base == 16 && c >= 'A' && c <= 'F') {
            d = 10 + (c - 'A');
        } else {
            return false;
        }
        if (magnitude > (std::numeric_limits<std::uint64_t>::max() - static_cast<std::uint64_t>(d)) / static_cast<std::uint64_t>(base)) {
            return false;  // overflow
        }
        magnitude = magnitude * static_cast<std::uint64_t>(base) + static_cast<std::uint64_t>(d);
    }
    return true;
}

bool is_signed(Type t) noexcept {
    return t == Type::I8 || t == Type::I16 || t == Type::I32 || t == Type::I64;
}

std::vector<std::byte> le_bytes(std::uint64_t v, std::size_t width) {
    std::vector<std::byte> out(width);
    for (std::size_t i = 0; i < width; ++i) {
        out[i] = static_cast<std::byte>((v >> (8U * i)) & 0xFFU);
    }
    return out;
}

std::vector<std::byte> parse_hex_bytes(std::string_view text) {
    std::vector<std::byte> out;
    unsigned acc = 0;
    int digits = 0;
    for (const char c : text) {
        if (c == ' ' || c == ':' || c == ',') {
            continue;
        }
        int d = 0;
        if (c >= '0' && c <= '9') {
            d = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            d = 10 + (c - 'a');
        } else if (c >= 'A' && c <= 'F') {
            d = 10 + (c - 'A');
        } else {
            throw Error("bytes value: '" + std::string(text) + "' is not a hex byte string");
        }
        acc = (acc << 4U) | static_cast<unsigned>(d);
        if (++digits == 2) {
            out.push_back(static_cast<std::byte>(acc));
            acc = 0;
            digits = 0;
        }
    }
    if (digits != 0) {
        throw Error("bytes value: odd number of hex digits");
    }
    return out;
}

}  // namespace

std::optional<Type> parse_type(std::string_view name) noexcept {
    static constexpr std::array<std::pair<std::string_view, Type>, 10> kNames{{
        {"u8", Type::U8},
        {"i8", Type::I8},
        {"u16", Type::U16},
        {"i16", Type::I16},
        {"u32", Type::U32},
        {"i32", Type::I32},
        {"u64", Type::U64},
        {"i64", Type::I64},
        {"string", Type::String},
        {"bytes", Type::Bytes},
    }};
    const auto l = lower(name);
    for (const auto& [n, t] : kNames) {
        if (l == n) {
            return t;
        }
    }
    return std::nullopt;
}

const char* to_string(Type type) noexcept {
    switch (type) {
        case Type::U8:
            return "u8";
        case Type::I8:
            return "i8";
        case Type::U16:
            return "u16";
        case Type::I16:
            return "i16";
        case Type::U32:
            return "u32";
        case Type::I32:
            return "i32";
        case Type::U64:
            return "u64";
        case Type::I64:
            return "i64";
        case Type::String:
            return "string";
        case Type::Bytes:
            return "bytes";
    }
    return "?";
}

std::size_t fixed_width(Type type) noexcept {
    switch (type) {
        case Type::U8:
        case Type::I8:
            return 1;
        case Type::U16:
        case Type::I16:
            return 2;
        case Type::U32:
        case Type::I32:
            return 4;
        case Type::U64:
        case Type::I64:
            return 8;
        case Type::String:
        case Type::Bytes:
            return 0;
    }
    return 0;
}

std::vector<std::byte> encode(Type type, const Input& value) {
    if (type == Type::String) {
        if (!value.text) {
            throw Error("string value must be given as a string");
        }
        std::vector<std::byte> out(value.text->size());
        for (std::size_t i = 0; i < out.size(); ++i) {
            out[i] = static_cast<std::byte>((*value.text)[i]);
        }
        return out;
    }
    if (type == Type::Bytes) {
        if (!value.text) {
            throw Error("bytes value must be given as a hex string, e.g. \"01 02 FF\"");
        }
        return parse_hex_bytes(*value.text);
    }
    // Integers: from a JSON number (must be integral and exact) or a decimal/hex string.
    bool negative = false;
    std::uint64_t magnitude = 0;
    if (value.number) {
        const double d = *value.number;
        if (!std::isfinite(d) || std::floor(d) != d) {
            throw Error("integer value must be a whole number (got " + std::to_string(d) + ")");
        }
        if (std::fabs(d) > kExactDouble) {
            throw Error("integer value exceeds 2^53; pass it as a string (\"0x...\" or decimal)");
        }
        negative = d < 0;
        magnitude = static_cast<std::uint64_t>(std::fabs(d));
    } else if (value.text) {
        if (!parse_integer(*value.text, negative, magnitude)) {
            throw Error("integer value '" + *value.text + "' is not a decimal or 0x-prefixed hex integer");
        }
    } else {
        throw Error("value is missing");
    }
    const std::size_t width = fixed_width(type);
    const unsigned bits = static_cast<unsigned>(width * 8U);
    if (is_signed(type)) {
        const std::uint64_t max_pos = (bits == 64) ? std::numeric_limits<std::int64_t>::max() : ((1ULL << (bits - 1U)) - 1ULL);
        const std::uint64_t max_neg = max_pos + 1ULL;
        if ((negative && magnitude > max_neg) || (!negative && magnitude > max_pos)) {
            throw Error(std::string("value out of range for ") + to_string(type));
        }
        const std::uint64_t twos = negative ? (~magnitude + 1ULL) : magnitude;  // two's complement, then truncate
        return le_bytes(twos, width);
    }
    if (negative) {
        throw Error(std::string("negative value for unsigned type ") + to_string(type));
    }
    const std::uint64_t max = (bits == 64) ? std::numeric_limits<std::uint64_t>::max() : ((1ULL << bits) - 1ULL);
    if (magnitude > max) {
        throw Error(std::string("value out of range for ") + to_string(type));
    }
    return le_bytes(magnitude, width);
}

Decoded decode(Type type, const std::vector<std::byte>& bytes) {
    Decoded d;
    if (type == Type::String) {
        for (const std::byte b : bytes) {
            const char c = static_cast<char>(b);
            if (c == '\0') {
                break;
            }
            d.text.push_back(c);
        }
        return d;
    }
    if (type == Type::Bytes) {
        d.text = hex_bytes(bytes);
        return d;
    }
    const std::size_t width = fixed_width(type);
    if (bytes.size() < width) {
        throw Error(std::string("drive returned ") + std::to_string(bytes.size()) + " byte(s), " + to_string(type) + " needs " +
                    std::to_string(width));
    }
    std::uint64_t raw = 0;
    for (std::size_t i = 0; i < width; ++i) {
        raw |= static_cast<std::uint64_t>(bytes[i]) << (8U * i);
    }
    if (is_signed(type)) {
        const unsigned bits = static_cast<unsigned>(width * 8U);
        std::int64_t v = 0;
        if (bits == 64) {
            v = static_cast<std::int64_t>(raw);
        } else {
            const std::uint64_t sign = 1ULL << (bits - 1U);
            v = (raw & sign) ? static_cast<std::int64_t>(raw) - static_cast<std::int64_t>(1ULL << bits) : static_cast<std::int64_t>(raw);
        }
        if (static_cast<double>(v) <= kExactDouble && static_cast<double>(v) >= -kExactDouble) {
            d.is_number = true;
            d.number = static_cast<double>(v);
        }
        d.text = std::to_string(v);
        return d;
    }
    if (static_cast<double>(raw) <= kExactDouble) {
        d.is_number = true;
        d.number = static_cast<double>(raw);
    }
    d.text = std::to_string(raw);
    return d;
}

std::string hex_bytes(const std::vector<std::byte>& bytes) {
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

}  // namespace ethercat::servo::sdo_codec
