#pragma once

// Reading Viam ProtoStruct values: config attributes and do_command arguments and replies. Shared
// by the components in this module so the two of them agree on what a missing key, a wrong type
// and a nested struct mean.

#include <optional>
#include <string>

#include <viam/sdk/common/proto_value.hpp>

#include "ethercat/errors.hpp"

namespace ethercat::servo::attrs {

using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;

inline const ProtoValue* find_attr(const ProtoStruct& s, const std::string& key) {
    const auto it = s.find(key);
    return it == s.end() ? nullptr : &it->second;
}

// Typed read; throws when the key is present with another type.
template <class T>
std::optional<T> opt_attr(const ProtoStruct& s, const std::string& key) {
    const ProtoValue* const v = find_attr(s, key);
    if (v == nullptr) {
        return std::nullopt;
    }
    const T* const p = v->get<T>();
    if (p == nullptr) {
        throw Error("config attribute '" + key + "' has the wrong type");
    }
    return *p;
}

inline double req_num(const ProtoStruct& s, const std::string& key) {
    const std::optional<double> v = opt_attr<double>(s, key);
    if (!v) {
        throw Error("required config attribute '" + key + "' is missing");
    }
    return *v;
}

inline double opt_num(const ProtoStruct& s, const std::string& key, double dflt) {
    return opt_attr<double>(s, key).value_or(dflt);
}

// A do_command key set to boolean true (the shape every flag verb takes).
inline bool flag(const ProtoStruct& s, const std::string& key) {
    const ProtoValue* const v = find_attr(s, key);
    const bool* const b = v == nullptr ? nullptr : v->get<bool>();
    return b != nullptr && *b;
}

// The nested struct under `key`, or nullptr: a verb's arguments, or its reply.
inline const ProtoStruct* nested(const ProtoStruct& s, const std::string& key) {
    const ProtoValue* const v = find_attr(s, key);
    return v == nullptr ? nullptr : v->get<ProtoStruct>();
}

}  // namespace ethercat::servo::attrs
