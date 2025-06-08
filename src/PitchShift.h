#pragma once

#include <math.h>
#include <stdio.h>
#include <string.h>

#include "AudioTools.h"

//pitch shift effect with improved interpolation and crossfading
class PitchShiftFixedOutput : public AudioOutput {
public:
  PitchShiftFixedOutput(Print &out) { p_out = &out; }

  PitchShiftInfo defaultConfig() {
    PitchShiftInfo result;
    result.bits_per_sample = sizeof(int16_t) * 8;
    result.pitch_shift = 1.0f;
    result.buffer_size = 1024;
    return result;
  }

  bool begin(PitchShiftInfo info);

  size_t write(const uint8_t *data, size_t len) override {
    size_t result = 0;
    int16_t *p_in = (int16_t *)data;
    int sample_count = len / sizeof(int16_t);

    for (int j = 0; j < sample_count; j += 1) {
      int16_t value = p_in[j];

      int16_t out_value = pitchShift(value);
      result += p_out->write((uint8_t *)&out_value, sizeof(int16_t));
    }
    return result;
  }

  void end() {}

protected:
  Print *p_out = nullptr;
  PitchShiftInfo cfg;  // Add missing configuration variable

  int16_t pitchShift(int16_t value);
  uint32_t pitchMul;
  unsigned long secondaryOffset;
  bool initialized = false;
};