#pragma once

// Typed encode/decode of CoE SDO payloads for the raw sdo_read/sdo_write do_command verbs. Values
// travel as JSON numbers (doubles, exact to 2^53) or strings ("0x..", decimal, hex bytes, text).

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ethercat::servo::sdo_codec {

enum class Type : std::uint8_t { U8, I8, U16, I16, U32, I32, U64, I64, String, Bytes };

std::optional<Type> parse_type(std::string_view name) noexcept;  // "u8", "i16", "string", "bytes", ...
const char* to_string(Type type) noexcept;
std::size_t fixed_width(Type type) noexcept;  // 0 for String/Bytes

// A value as it arrives in a do_command: a number or a string, never both.
struct Input {
    std::optional<double> number;
    std::optional<std::string> text;
};

// Little-endian bytes for `type`. Throws ethercat::Error on a missing/ill-typed/out-of-range value.
std::vector<std::byte> encode(Type type, const Input& value);

struct Decoded {
    bool is_number = false;  // integers within +-2^53 are reported as a number, otherwise as text
    double number = 0.0;
    std::string text;  // decimal for large integers, the text for String, "01 02 FF" for Bytes
};
Decoded decode(Type type, const std::vector<std::byte>& bytes);

std::string hex_bytes(const std::vector<std::byte>& bytes);  // "01 02 FF"

}  // namespace ethercat::servo::sdo_codec
