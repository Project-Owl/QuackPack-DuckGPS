#pragma once
#include <cstdint>
#include <cstdlib>
#include <cmath>
using byte = uint8_t;
extern uint32_t fakeMillis;
extern unsigned delayCalls;
inline uint32_t millis() { return fakeMillis; }
inline void delay(unsigned long ms) { ++delayCalls; fakeMillis += uint32_t(ms); }
constexpr int SERIAL_8N1 = 0;
constexpr double TWO_PI = 6.2831853071795864769;
inline double radians(double d) { return d * TWO_PI / 360; }
inline double degrees(double r) { return r * 360 / TWO_PI; }
inline double sq(double v) { return v*v; }
