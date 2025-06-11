#ifndef VAD_CONFIG_H
#define VAD_CONFIG_H

// VAD Configuration Constants
// These can be adjusted based on your microphone and environment

// Energy thresholds (may need tuning based on your microphone sensitivity)
// Higher values = less sensitive (fewer false positives)
// Lower values = more sensitive (may pick up background noise)
#define VAD_SPEECH_THRESHOLD    1450.0f    // RMS energy threshold for speech start
#define VAD_SILENCE_THRESHOLD   1400.0f    // RMS energy threshold for speech end (hysteresis)

// Timing parameters (in frames, where each frame = 20ms)
#define VAD_PREFIX_FRAMES       5         // 200ms prefix padding (10 * 20ms)
#define VAD_SILENCE_FRAMES      10        // 800ms silence detection (10 * 20ms)
#define VAD_MIN_SPEECH_FRAMES   5         // 100ms minimum speech duration (5 * 20ms)

// Audio parameters - 20ms frames at 16kHz
#define VAD_SAMPLE_RATE         16000     // 16kHz sample rate
#define VAD_FRAME_SIZE          320       // 20ms frame at 16kHz (320 samples)

// Smoothing
#define VAD_ENERGY_SMOOTHING    0.1f      // Low-pass filter factor for energy smoothing

// Debug settings - using const instead of define to avoid conflicts
static const unsigned long VAD_DEBUG_INTERVAL_MS = 10000;  // Debug output every 10 seconds (milliseconds)

// Microphone calibration notes:
// - If VAD is too sensitive (picks up background noise):
//   Increase VAD_SPEECH_THRESHOLD and VAD_SILENCE_THRESHOLD
// - If VAD misses quiet speech:
//   Decrease VAD_SPEECH_THRESHOLD and VAD_SILENCE_THRESHOLD
// - If speech gets cut off at the beginning:
//   Increase VAD_PREFIX_FRAMES
// - If speech gets cut off at the end:
//   Increase VAD_SILENCE_FRAMES
// - If short words get ignored:
//   Decrease VAD_MIN_SPEECH_FRAMES

#endif // VAD_CONFIG_H
