// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/json.h
//
//  Minimal, allocation-frugal JSON reader/writer.  Needed because the pipeline
//  ingests GeoJSON (BMC wards 1 MB, slum clusters 3.4 MB, SRA 7.7 MB) and the
//  runtime emits manifests, and the project's dependency policy (DOCS D-014)
//  forbids pulling a third-party C++ library in for this.  Handles the full
//  JSON grammar including exponents and \u escapes.
// =============================================================================
#pragma once
#include "engine/core/core.h"
#include <map>

namespace mumbai {

class Json {
public:
    enum class Type { Null, Bool, Number, String, Array, Object };

    Json() : m_type(Type::Null) {}
    Json(bool b) : m_type(Type::Bool), m_bool(b) {}
    Json(f64 n) : m_type(Type::Number), m_number(n) {}
    Json(i64 n) : m_type(Type::Number), m_number(static_cast<f64>(n)) {}
    Json(int n) : m_type(Type::Number), m_number(static_cast<f64>(n)) {}
    Json(const char* s) : m_type(Type::String), m_string(s) {}
    Json(std::string s) : m_type(Type::String), m_string(std::move(s)) {}

    static Json array() { Json j; j.m_type = Type::Array; return j; }
    static Json object() { Json j; j.m_type = Type::Object; return j; }

    Type type() const { return m_type; }
    bool isNull()   const { return m_type == Type::Null; }
    bool isBool()   const { return m_type == Type::Bool; }
    bool isNumber() const { return m_type == Type::Number; }
    bool isString() const { return m_type == Type::String; }
    bool isArray()  const { return m_type == Type::Array; }
    bool isObject() const { return m_type == Type::Object; }

    bool asBool(bool def = false) const { return m_type == Type::Bool ? m_bool : def; }
    f64 asNumber(f64 def = 0.0) const { return m_type == Type::Number ? m_number : def; }
    i64 asInt(i64 def = 0) const {
        if (m_type == Type::Number) return static_cast<i64>(m_number);
        if (m_type == Type::Bool) return m_bool ? 1 : 0;
        return def;
    }
    const std::string& asString() const { return m_string; }
    std::string asStringOr(const std::string& def) const {
        return m_type == Type::String ? m_string : (m_type == Type::Number ? std::to_string(m_number) : def);
    }

    // Container access
    size_t size() const {
        if (m_type == Type::Array) return m_array.size();
        if (m_type == Type::Object) return m_object.size();
        return 0;
    }
    const Json& operator[](size_t i) const {
        static const Json nullValue;
        return (m_type == Type::Array && i < m_array.size()) ? m_array[i] : nullValue;
    }
    const Json& operator[](const std::string& key) const {
        static const Json nullValue;
        if (m_type != Type::Object) return nullValue;
        auto it = m_object.find(key);
        return it == m_object.end() ? nullValue : it->second;
    }
    bool has(const std::string& key) const {
        return m_type == Type::Object && m_object.count(key) > 0;
    }
    const std::vector<Json>& items() const { return m_array; }
    const std::map<std::string, Json>& fields() const { return m_object; }

    // Builders
    void push(Json v) { m_type = Type::Array; m_array.push_back(std::move(v)); }
    void set(const std::string& key, Json v) {
        m_type = Type::Object;
        m_object[key] = std::move(v);
    }
    Json& operator[](const std::string& key) {
        m_type = Type::Object;
        return m_object[key];
    }

    // Mutating array helper used by the manifest/export stages.
    void setArray(const std::string& key, Json arr) {
        m_type = Type::Object;
        m_object[key] = std::move(arr);
    }

    std::string dump(int indent = -1) const;

    // Parses `text`.  Returns true on success; `error` receives a diagnostic
    // including byte offset on failure.
    static bool parse(std::string_view text, Json& out, std::string* error = nullptr);

private:
    void dumpTo(std::string& out, int indent, int depth) const;

    Type m_type = Type::Null;
    bool m_bool = false;
    f64 m_number = 0.0;
    std::string m_string;
    std::vector<Json> m_array;
    std::map<std::string, Json> m_object;
};

} // namespace mumbai
