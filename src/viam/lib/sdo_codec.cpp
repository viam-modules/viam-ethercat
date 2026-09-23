#include "viam/lib/sdo_codec.hpp"

#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <limits>

#include "ethercat/errors.hpp"
#include "ethercat/log.hpp"
#include "ethercat/pdo_buffer.hpp"

namespace ethercat::servo::sdo_codec {

namespace {

std::int64_t to_integer(const Input& value) {
    if (value.number) {
        if (!std::isfinite(*value.number) || std::floor(*value.number) != *value.number) {
            throw Error("integer value must be a whole number");
        }
        return static_cast<std::int64_t>(*value.number);
    }
    if (!value.text) {
        throw Error("value is missing");
    }
    std::string_view s = *value.text;
    const bool neg = s.starts_with('-');
    s.remove_prefix(neg ? 1 : 0);
    const bool hex = s.starts_with("0x") || s.starts_with("0X");
    s.remove_prefix(hex ? 2 : 0);
    std::int64_t v = 0;
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v, hex ? 16 : 10);
    if (s.empty() || ec != std::errc{} || end != s.data() + s.size()) {
        throw Error("integer value '" + *value.text + "' is not a decimal or 0x-prefixed integer");
    }
    return neg ? -v : v;
}

template <class T>
std::vector<std::byte> encode_int(const Input& value) {
    const std::int64_t v = to_integer(value);
    if (v < std::numeric_limits<T>::min() || v > std::numeric_limits<T>::max()) {
        throw Error("value " + std::to_string(v) + " out of range [" + std::to_string(std::numeric_limits<T>::min()) + ", " +
                    std::to_string(std::numeric_limits<T>::max()) + "]");
    }
    std::vector<std::byte> out(sizeof(T));
    store_le<T>(out, static_cast<T>(v));
    return out;
}

template <class T>
Decoded decode_int(const std::vector<std::byte>& bytes) {
    if (bytes.size() < sizeof(T)) {
        throw Error("drive returned " + std::to_string(bytes.size()) + " byte(s), expected " + std::to_string(sizeof(T)));
    }
    return {static_cast<double>(load_le<T>(bytes)), {}};
}

const std::string& text_of(const Input& value) {
    if (!value.text) {
        throw Error("string/bytes value must be a string");
    }
    return *value.text;
}

std::vector<std::byte> encode_string(const Input& value) {
    std::vector<std::byte> out;
    for (const char c : text_of(value)) {
        out.push_back(static_cast<std::byte>(c));
    }
    return out;
}

Decoded decode_string(const std::vector<std::byte>& bytes) {
    Decoded d;
    for (const std::byte b : bytes) {
        if (b == std::byte{0}) {
            break;
        }
        d.text.push_back(static_cast<char>(b));
    }
    return d;
}

std::vector<std::byte> encode_bytes(const Input& value) {
    std::string digits;
    for (const char c : text_of(value)) {
        if (c != ' ') {
            digits.push_back(c);
        }
    }
    if (digits.size() % 2 != 0) {
        throw Error("bytes value must have an even number of hex digits");
    }
    std::vector<std::byte> out;
    for (std::size_t i = 0; i < digits.size(); i += 2) {
        unsigned v = 0;
        const auto [end, ec] = std::from_chars(digits.data() + i, digits.data() + i + 2, v, 16);
        if (ec != std::errc{} || end != digits.data() + i + 2) {
            throw Error("bytes value '" + *value.text + "' is not a hex byte string");
        }
        out.push_back(static_cast<std::byte>(v));
    }
    return out;
}

Decoded decode_bytes(const std::vector<std::byte>& bytes) {
    return {std::nullopt, log::hex_bytes(bytes)};
}

constexpr std::array<TypeInfo, 8> kTypes{{
    {"u8", 1, &encode_int<std::uint8_t>, &decode_int<std::uint8_t>},
    {"i8", 1, &encode_int<std::int8_t>, &decode_int<std::int8_t>},
    {"u16", 2, &encode_int<std::uint16_t>, &decode_int<std::uint16_t>},
    {"i16", 2, &encode_int<std::int16_t>, &decode_int<std::int16_t>},
    {"u32", 4, &encode_int<std::uint32_t>, &decode_int<std::uint32_t>},
    {"i32", 4, &encode_int<std::int32_t>, &decode_int<std::int32_t>},
    {"string", 0, &encode_string, &decode_string},
    {"bytes", 0, &encode_bytes, &decode_bytes},
}};

}  // namespace

const TypeInfo* find_type(std::string_view name) noexcept {
    const auto* const it = std::find_if(kTypes.begin(), kTypes.end(), [name](const TypeInfo& t) { return t.name == name; });
    return it == kTypes.end() ? nullptr : &*it;
}

}  // namespace ethercat::servo::sdo_codec
