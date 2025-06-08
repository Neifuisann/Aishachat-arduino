#include "PitchShift.h"

#define GRAINSIZE 1024ul
static int16_t buf1[GRAINSIZE] = {0};  // Initialize to zero
static int16_t buf2[GRAINSIZE] = {0};  // Initialize to zero
static int16_t* buf = buf1;
static int16_t* buf_ = buf2;
static unsigned long readAddress = 0;
static unsigned long writeAddress = 0;
static bool buffersInitialized = false;

bool PitchShiftFixedOutput::begin(PitchShiftInfo info) {
  TRACED();
  cfg = info;
  AudioOutput::setAudioInfo(info);

  // Initialize buffers to zero if not already done
  if (!buffersInitialized) {
    memset(buf1, 0, sizeof(buf1));
    memset(buf2, 0, sizeof(buf2));
    buffersInitialized = true;
  }

  // Reset addresses
  readAddress = 0;
  writeAddress = 0;
  buf = buf1;
  buf_ = buf2;

  // Calculate pitch multiplier with better precision
  this->pitchMul = (uint32_t)(info.pitch_shift * 256.0f + 0.5f);
  this->secondaryOffset = GRAINSIZE - ((( this->pitchMul * GRAINSIZE ) >> 8 ) % GRAINSIZE);

  initialized = true;
  return true;
}

int16_t PitchShiftFixedOutput::pitchShift(int16_t value) {
  if (!initialized) {
    return value;  // Pass through if not initialized
  }

  // Store input sample
  buf_[writeAddress] = value;

  // Calculate read positions with better precision
  int ii1 = (writeAddress * this->pitchMul) >> 8;
  int output1 = buf[ii1 % GRAINSIZE];
  int ii2 = ii1 + secondaryOffset;
  int output2 = buf[ii2 % GRAINSIZE];

  // Improved crossfading with smoother transitions
  unsigned long f = 0;
  if (writeAddress >= GRAINSIZE * 3 / 4) {
    // Full crossfade in last quarter
    f = GRAINSIZE;
  } else if (writeAddress >= GRAINSIZE / 4) {
    // Gradual crossfade in middle half with smoother curve
    unsigned long pos = writeAddress - GRAINSIZE / 4;
    unsigned long range = GRAINSIZE / 2;
    // Use a smoother curve (sine-like approximation)
    f = (pos * pos * 4) / range;  // Quadratic curve for smoother transition
    if (f > GRAINSIZE) f = GRAINSIZE;
  }

  // Mix the two grains with crossfading
  int output = (output1 * (GRAINSIZE - f) + output2 * f) / GRAINSIZE;

  // Apply gentle low-pass filtering to reduce high-frequency artifacts
  static int16_t lastOutput = 0;
  output = (output * 3 + lastOutput) / 4;  // Simple 1-pole low-pass
  lastOutput = output;

  writeAddress++;
  if (writeAddress >= GRAINSIZE) {
    writeAddress = 0; // loop around to beginning of grain
    readAddress = 0;

    // Swap buffers with smooth transition
    buf_ = buf;
    buf = (buf == buf1) ? buf2 : buf1;
  }

  // Clamp output to prevent overflow
  if (output > 32767) output = 32767;
  if (output < -32768) output = -32768;

  return (int16_t)output;
}