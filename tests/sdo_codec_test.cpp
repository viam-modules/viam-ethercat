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
    return codec::Input{d, std::nullopt};
}
codec::Input txt(const char* s) {
    return codec::Input{std::nullopt, s};
}
const codec::TypeInfo& T(const char* name) {
    const codec::TypeInfo* t = codec::find_type(name);
    CHECK(t != nullptr);
    return *t;
}
}  // namespace

TEST("integers: little-endian from a number or a 0x string, sign and range checked") {
    CHECK(codec::find_type("f32") == nullptr);
    CHECK(T("u16").encode(num(0x1234)) == bytes({0x34, 0x12}));
    CHECK(T("i8").encode(txt("-4")) == bytes({0xFC}));
    CHECK(T("u32").encode(txt("0x65766173")) == bytes({0x73, 0x61, 0x76, 0x65}));
    CHECK_THROWS_MSG(T("u8").encode(num(256)), Error, "out of range");
    CHECK_THROWS_MSG(T("u8").encode(num(-1)), Error, "out of range");
    CHECK_THROWS_MSG(T("i16").encode(num(1.5)), Error, "whole number");
    CHECK_THROWS_MSG(T("u16").encode(txt("12z")), Error, "not a decimal");
}

TEST("strings and hex byte strings encode and decode") {
    CHECK(T("bytes").encode(txt("01 02 ff")) == bytes({0x01, 0x02, 0xFF}));
    CHECK_THROWS_MSG(T("bytes").encode(txt("abc")), Error, "even number");
    CHECK(T("string").encode(txt("M5")) == bytes({'M', '5'}));
    CHECK_EQ(T("string").decode(bytes({'A', 'B', 0, 'x'})).text, std::string("AB"));
    CHECK_EQ(T("bytes").decode(bytes({0x00, 0xAB})).text, std::string("00 AB"));
}

TEST("integers decode with sign extension; a short reply is an error") {
    CHECK_EQ(*T("i16").decode(bytes({0xFC, 0xFF})).number, -4.0);
    CHECK_EQ(*T("u32").decode(bytes({0xFF, 0xFF, 0xFF, 0xFF})).number, 4294967295.0);
    CHECK_THROWS_MSG(T("u32").decode(bytes({0x01})), Error, "expected 4");
}

TEST_MAIN()
