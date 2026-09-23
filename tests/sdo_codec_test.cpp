// sdo_codec: typed encode/decode for the raw SDO do_command verbs.

#include <cstddef>
#include <string>
#include <vector>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/lib/sdo_codec.hpp"

using ethercat::Error;
namespace codec = ethercat::servo::sdo_codec;

namespace {
std::vector<std::byte> bytes(std::initializer_list<int> v) {
    std::vector<std::byte> out;
    for (const int b : v) {
        out.push_back(static_cast<std::byte>(b));
    }
    return out;
}
codec::Input num(double d) {
    codec::Input in;
    in.number = d;
    return in;
}
codec::Input txt(const char* s) {
    codec::Input in;
    in.text = s;
    return in;
}
}  // namespace

TEST("type names parse case-insensitively; unknown names are rejected") {
    CHECK(codec::parse_type("U16").has_value());
    CHECK_EQ(static_cast<int>(*codec::parse_type("i8")), static_cast<int>(codec::Type::I8));
    CHECK(!codec::parse_type("float").has_value());
    CHECK_EQ(codec::fixed_width(codec::Type::U32), std::size_t{4});
    CHECK_EQ(codec::fixed_width(codec::Type::Bytes), std::size_t{0});
}

TEST("integers encode little-endian from numbers or 0x strings, with range checks") {
    CHECK(codec::encode(codec::Type::U16, num(0x1234)) == bytes({0x34, 0x12}));
    CHECK(codec::encode(codec::Type::I8, num(-4)) == bytes({0xFC}));
    CHECK(codec::encode(codec::Type::I32, txt("-1")) == bytes({0xFF, 0xFF, 0xFF, 0xFF}));
    CHECK(codec::encode(codec::Type::U32, txt("0x65766173")) == bytes({0x73, 0x61, 0x76, 0x65}));  // "save"
    CHECK_THROWS_MSG(codec::encode(codec::Type::U8, num(256)), Error, "out of range");
    CHECK_THROWS_MSG(codec::encode(codec::Type::U8, num(-1)), Error, "negative");
    CHECK_THROWS_MSG(codec::encode(codec::Type::I16, num(1.5)), Error, "whole number");
    CHECK_THROWS_MSG(codec::encode(codec::Type::U16, txt("12z")), Error, "not a decimal");
    CHECK_THROWS_MSG(codec::encode(codec::Type::U16, codec::Input{}), Error, "missing");
}

TEST("strings and byte strings round-trip") {
    CHECK(codec::encode(codec::Type::Bytes, txt("01 02 ff")) == bytes({0x01, 0x02, 0xFF}));
    CHECK_THROWS_MSG(codec::encode(codec::Type::Bytes, txt("abc")), Error, "odd number");
    CHECK(codec::encode(codec::Type::String, txt("M56S")) == bytes({'M', '5', '6', 'S'}));
    CHECK_EQ(codec::decode(codec::Type::String, bytes({'A', 'B', 0, 'x'})).text, std::string("AB"));
    CHECK_EQ(codec::decode(codec::Type::Bytes, bytes({0x00, 0xAB})).text, std::string("00 AB"));
}

TEST("decode reports exact integers as numbers and huge ones as text") {
    const auto d = codec::decode(codec::Type::I16, bytes({0xFC, 0xFF}));
    CHECK(d.is_number);
    CHECK_EQ(d.number, -4.0);
    const auto u = codec::decode(codec::Type::U64, bytes({0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF}));
    CHECK(!u.is_number);
    CHECK_EQ(u.text, std::string("18446744073709551615"));
    CHECK_THROWS_MSG(codec::decode(codec::Type::U32, bytes({0x01})), Error, "needs 4");
}

TEST_MAIN()
