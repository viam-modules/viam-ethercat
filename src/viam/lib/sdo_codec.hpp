#pragma once

// Typed CoE SDO payloads for the raw sdo_read/sdo_write do_command verbs.

#include <cstddef>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace ethercat::servo::sdo_codec {

// A do_command value: a JSON number or a string ("0x1F", "-3", hex bytes "01 FF", or text).
struct Input {
    std::optional<double> number;
    std::optional<std::string> text;
};

// Integers decode to `number`; string/bytes decode to `text` (text, or "01 FF").
struct Decoded {
    std::optional<double> number;
    std::string text;
};

struct TypeInfo {
    std::string_view name;
    std::size_t width;                                 // 0 = variable (string, bytes)
    std::vector<std::byte> (*encode)(const Input&);    // little-endian; throws Error when invalid
    Decoded (*decode)(const std::vector<std::byte>&);  // throws Error on a short reply
};

const TypeInfo* find_type(std::string_view name) noexcept;  // "u8" "i8" "u16" "i16" "u32" "i32" "string" "bytes"

}  // namespace ethercat::servo::sdo_codec
