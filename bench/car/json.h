// Tiny JSON reader for the virtual car's control datagrams and profile file.
// Enough for objects, arrays, numbers, strings (\" \\ \n escapes), true/false/null.
#pragma once
#include <cctype>
#include <cstdlib>
#include <cstring>
#include <string>
#include <utility>
#include <vector>

struct JVal {
  enum T { NUL, BOOL, NUM, STR, ARR, OBJ } t = NUL;
  bool b = false;
  double n = 0;
  std::string s;
  std::vector<JVal> a;
  std::vector<std::pair<std::string, JVal>> o;

  const JVal *get(const std::string &k) const {
    if (t != OBJ) return nullptr;
    for (auto &kv : o) if (kv.first == k) return &kv.second;
    return nullptr;
  }
  bool isNum() const { return t == NUM; }
  double num(double dflt = 0) const { return t == NUM ? n : t == BOOL ? (b ? 1 : 0) : dflt; }
};

class JParser {
  const char *p, *e;
  bool ok = true;
  void ws() { while (p < e && isspace((unsigned char)*p)) p++; }
  bool lit(const char *w) {
    size_t n = strlen(w);
    if ((size_t)(e - p) >= n && !strncmp(p, w, n)) { p += n; return true; }
    return false;
  }
  std::string str() {
    std::string out;
    p++;  // opening quote
    while (p < e && *p != '"') {
      if (*p == '\\' && p + 1 < e) {
        p++;
        switch (*p) {
          case 'n': out += '\n'; break;
          case 't': out += '\t'; break;
          case 'r': out += '\r'; break;
          case 'u': out += '?'; p += (e - p > 4) ? 4 : 0; break;  // not needed here
          default: out += *p;
        }
        p++;
      } else {
        out += *p++;
      }
    }
    if (p >= e) { ok = false; return out; }
    p++;
    return out;
  }
  JVal val(int depth) {
    JVal v;
    ws();
    if (p >= e || depth > 32) { ok = false; return v; }
    if (*p == '{') {
      v.t = JVal::OBJ; p++; ws();
      if (p < e && *p == '}') { p++; return v; }
      while (ok) {
        ws();
        if (p >= e || *p != '"') { ok = false; break; }
        std::string k = str();
        ws();
        if (p >= e || *p != ':') { ok = false; break; }
        p++;
        JVal c = val(depth + 1);
        v.o.emplace_back(k, std::move(c));
        ws();
        if (p < e && *p == ',') { p++; continue; }
        if (p < e && *p == '}') { p++; break; }
        ok = false;
      }
    } else if (*p == '[') {
      v.t = JVal::ARR; p++; ws();
      if (p < e && *p == ']') { p++; return v; }
      while (ok) {
        v.a.push_back(val(depth + 1));
        ws();
        if (p < e && *p == ',') { p++; continue; }
        if (p < e && *p == ']') { p++; break; }
        ok = false;
      }
    } else if (*p == '"') {
      v.t = JVal::STR; v.s = str();
    } else if (lit("true")) { v.t = JVal::BOOL; v.b = true; }
    else if (lit("false")) { v.t = JVal::BOOL; v.b = false; }
    else if (lit("null")) { v.t = JVal::NUL; }
    else {
      std::string num(p, (size_t)(e - p) < 64 ? (size_t)(e - p) : 64);
      char *end;
      double d = strtod(num.c_str(), &end);
      if (end == num.c_str()) { ok = false; return v; }
      p += end - num.c_str();
      v.t = JVal::NUM; v.n = d;
    }
    return v;
  }

 public:
  // Returns false on malformed input.
  static bool parse(const std::string &text, JVal &out) {
    JParser j;
    j.p = text.data(); j.e = text.data() + text.size();
    out = j.val(0);
    j.ws();
    return j.ok && j.p == j.e;
  }
};

inline std::string jesc(const std::string &s) {
  std::string o;
  for (char c : s) {
    if (c == '"' || c == '\\') { o += '\\'; o += c; }
    else if (c == '\n') o += "\\n";
    else if ((unsigned char)c < 0x20) o += ' ';
    else o += c;
  }
  return o;
}
