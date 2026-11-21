#pragma once
#include <string>
template <class T> struct AbslFlag { T v; };
#define ABSL_FLAG(T, name, def, help) AbslFlag<T> FLAGS_##name{def}
namespace absl { template <class T> T GetFlag(const AbslFlag<T>& f) { return f.v; } }
