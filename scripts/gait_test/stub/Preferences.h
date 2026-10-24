// Preferences does nothing here: gait_test uses the defaults.
#pragma once
#include <cstddef>
struct Preferences {
  bool begin(const char*, bool) { return true; } void end() {}
  size_t getBytesLength(const char*) { return 0; } size_t getBytes(const char*, void*, size_t) { return 0; }
  bool getBool(const char*, bool d) { return d; } uint8_t getUChar(const char*, uint8_t d) { return d; }
  void putBytes(const char*, const void*, size_t) {} void putBool(const char*, bool) {} void putUChar(const char*, uint8_t) {}
};
