// Just enough of Arduino.h for ik_gait.h / gaits.h to build on a PC (gait_test).
#pragma once
#define _USE_MATH_DEFINES
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include <cmath>
#include <cstdint>
#include <cstring>
#include <cstdio>
#include <algorithm>
using std::max; using std::min;
template<class T, class L, class H> T constrain(T x, L lo, H hi) { return x < lo ? (T)lo : (x > hi ? (T)hi : x); }
