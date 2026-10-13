// =============================================================================
//  MUMBAI 1:1 EXACT — engine/core/json.cpp
// =============================================================================
#include "engine/core/json.h"

namespace mumbai {
namespace {

struct Parser {
    std::string_view s;
    size_t i = 0;
    std::string err;

    bool eof() const { return i >= s.size(); }
    char peek() const { return i < s.size() ? s[i] : '\0'; }

    void skipWs() {
        while (i < s.size()) {
            const char c = s[i];
            if (c == ' ' || c == '\t' || c == '\n' || c == '\r') { ++i; continue; }
            break;
        }
    }
    bool fail(const char* msg) {
        if (err.empty()) {
            err = std::string(msg) + " at byte " + std::to_string(i);
        }
        return false;
    }

    bool parseValue(Json& out) {
        skipWs();
        if (eof()) return fail("unexpected end of input");
        const char c = peek();
        switch (c) {
            case '{': return parseObject(out);
            case '[': return parseArray(out);
            case '"': {
                std::string str;
                if (!parseString(str)) return false;
                out = Json(std::move(str));
                return true;
            }
            case 't':
                if (s.substr(i, 4) == "true") { i += 4; out = Json(true); return true; }
                return fail("invalid literal");
            case 'f':
                if (s.substr(i, 5) == "false") { i += 5; out = Json(false); return true; }
                return fail("invalid literal");
            case 'n':
                if (s.substr(i, 4) == "null") { i += 4; out = Json(); return true; }
                return fail("invalid literal");
            default: return parseNumber(out);
        }
    }

    bool parseNumber(Json& out) {
        const size_t start = i;
        if (peek() == '-') ++i;
        while (!eof() && (std::isdigit(static_cast<unsigned char>(peek())) || peek() == '.' ||
                          peek() == 'e' || peek() == 'E' || peek() == '+' || peek() == '-')) ++i;
        if (i == start) return fail("expected number");
        std::string tmp(s.substr(start, i - start));
        char* endp = nullptr;
        const f64 v = std::strtod(tmp.c_str(), &endp);
        if (endp == tmp.c_str()) return fail("malformed number");
        out = Json(v);
        return true;
    }

    bool parseString(std::string& out) {
        if (peek() != '"') return fail("expected string");
        ++i;
        out.clear();
        while (i < s.size()) {
            const char c = s[i++];
            if (c == '"') return true;
            if (c == '\\') {
                if (i >= s.size()) return fail("truncated escape");
                const char e = s[i++];
                switch (e) {
                    case '"': out.push_back('"'); break;
                    case '\\': out.push_back('\\'); break;
                    case '/': out.push_back('/'); break;
                    case 'b': out.push_back('\b'); break;
                    case 'f': out.push_back('\f'); break;
                    case 'n': out.push_back('\n'); break;
                    case 'r': out.push_back('\r'); break;
                    case 't': out.push_back('\t'); break;
                    case 'u': {
                        if (i + 4 > s.size()) return fail("truncated \\u escape");
                        unsigned cp = 0;
                        for (int k = 0; k < 4; ++k) {
                            const char h = s[i + k];
                            cp <<= 4;
                            if (h >= '0' && h <= '9') cp |= static_cast<unsigned>(h - '0');
                            else if (h >= 'a' && h <= 'f') cp |= static_cast<unsigned>(h - 'a' + 10);
                            else if (h >= 'A' && h <= 'F') cp |= static_cast<unsigned>(h - 'A' + 10);
                            else return fail("bad hex in \\u escape");
                        }
                        i += 4;
                        // Surrogate pair
                        if (cp >= 0xD800 && cp <= 0xDBFF && i + 6 <= s.size() &&
                            s[i] == '\\' && s[i + 1] == 'u') {
                            unsigned lo = 0;
                            bool ok = true;
                            for (int k = 0; k < 4; ++k) {
                                const char h = s[i + 2 + k];
                                lo <<= 4;
                                if (h >= '0' && h <= '9') lo |= static_cast<unsigned>(h - '0');
                                else if (h >= 'a' && h <= 'f') lo |= static_cast<unsigned>(h - 'a' + 10);
                                else if (h >= 'A' && h <= 'F') lo |= static_cast<unsigned>(h - 'A' + 10);
                                else { ok = false; break; }
                            }
                            if (ok) {
                                i += 6;
                                cp = 0x10000u + ((cp - 0xD800u) << 10) + (lo - 0xDC00u);
                            }
                        }
                        // UTF-8 encode
                        if (cp < 0x80) out.push_back(static_cast<char>(cp));
                        else if (cp < 0x800) {
                            out.push_back(static_cast<char>(0xC0 | (cp >> 6)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        } else if (cp < 0x10000) {
                            out.push_back(static_cast<char>(0xE0 | (cp >> 12)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        } else {
                            out.push_back(static_cast<char>(0xF0 | (cp >> 18)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 12) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | ((cp >> 6) & 0x3F)));
                            out.push_back(static_cast<char>(0x80 | (cp & 0x3F)));
                        }
                        break;
                    }
                    default: return fail("unknown escape");
                }
            } else {
                out.push_back(c);
            }
        }
        return fail("unterminated string");
    }

    bool parseArray(Json& out) {
        ++i;   // '['
        out = Json::array();
        skipWs();
        if (peek() == ']') { ++i; return true; }
        while (true) {
            Json v;
            if (!parseValue(v)) return false;
            out.push(std::move(v));
            skipWs();
            if (peek() == ',') { ++i; continue; }
            if (peek() == ']') { ++i; return true; }
            return fail("expected ',' or ']'");
        }
    }

    bool parseObject(Json& out) {
        ++i;   // '{'
        out = Json::object();
        skipWs();
        if (peek() == '}') { ++i; return true; }
        while (true) {
            skipWs();
            std::string key;
            if (!parseString(key)) return false;
            skipWs();
            if (peek() != ':') return fail("expected ':'");
            ++i;
            Json v;
            if (!parseValue(v)) return false;
            out.set(key, std::move(v));
            skipWs();
            if (peek() == ',') { ++i; continue; }
            if (peek() == '}') { ++i; return true; }
            return fail("expected ',' or '}'");
        }
    }
};

void escapeInto(const std::string& in, std::string& out) {
    out.push_back('"');
    for (char c : in) {
        switch (c) {
            case '"':  out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n";  break;
            case '\r': out += "\\r";  break;
            case '\t': out += "\\t";  break;
            case '\b': out += "\\b";  break;
            case '\f': out += "\\f";  break;
            default:
                if (static_cast<unsigned char>(c) < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof(buf), "\\u%04x", c);
                    out += buf;
                } else {
                    out.push_back(c);
                }
        }
    }
    out.push_back('"');
}

} // namespace

bool Json::parse(std::string_view text, Json& out, std::string* error) {
    Parser p{text, 0, {}};
    if (!p.parseValue(out)) {
        if (error) *error = p.err;
        return false;
    }
    p.skipWs();
    if (!p.eof()) {
        if (error) *error = "trailing data at byte " + std::to_string(p.i);
        return false;
    }
    return true;
}

void Json::dumpTo(std::string& out, int indent, int depth) const {
    auto newlineIndent = [&](int d) {
        if (indent < 0) return;
        out.push_back('\n');
        out.append(static_cast<size_t>(indent * d), ' ');
    };

    switch (m_type) {
        case Type::Null:   out += "null"; break;
        case Type::Bool:   out += m_bool ? "true" : "false"; break;
        case Type::Number: {
            char buf[64];
            if (m_number == std::floor(m_number) && std::fabs(m_number) < 1e15)
                std::snprintf(buf, sizeof(buf), "%lld", static_cast<long long>(m_number));
            else
                std::snprintf(buf, sizeof(buf), "%.9g", m_number);
            out += buf;
            break;
        }
        case Type::String: escapeInto(m_string, out); break;
        case Type::Array: {
            out.push_back('[');
            for (size_t k = 0; k < m_array.size(); ++k) {
                if (k) out.push_back(',');
                newlineIndent(depth + 1);
                m_array[k].dumpTo(out, indent, depth + 1);
            }
            if (!m_array.empty()) newlineIndent(depth);
            out.push_back(']');
            break;
        }
        case Type::Object: {
            out.push_back('{');
            bool first = true;
            for (const auto& [k, v] : m_object) {
                if (!first) out.push_back(',');
                first = false;
                newlineIndent(depth + 1);
                escapeInto(k, out);
                out.push_back(':');
                if (indent >= 0) out.push_back(' ');
                v.dumpTo(out, indent, depth + 1);
            }
            if (!m_object.empty()) newlineIndent(depth);
            out.push_back('}');
            break;
        }
    }
}

std::string Json::dump(int indent) const {
    std::string out;
    out.reserve(1 << 16);
    dumpTo(out, indent, 0);
    return out;
}

} // namespace mumbai
