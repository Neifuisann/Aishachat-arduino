#ifndef ADPCM_H
#define ADPCM_H

#include <Arduino.h>
#include <stdint.h>

/**
 * ADPCM (Adaptive Differential Pulse Code Modulation) Implementation
 * 
 * This implementation provides 4:1 compression ratio for 16-bit PCM audio.
 * Optimized for VoIP applications in bandwidth-constrained environments.
 * 
 * Features:
 * - 16-bit PCM input → 4-bit ADPCM output
 * - Maintains 16kHz sample rate
 * - Low latency encoding/decoding
 * - Minimal memory footprint
 */

class ADPCMEncoder {
private:
    int16_t valprev;     // Previous output value
    int8_t index;        // Index into stepsize table
    
    // ADPCM step size table
    static const int16_t stepsizeTable[89];
    
    // Index adjustment table
    static const int8_t indexTable[16];
    
    // Encode a single 16-bit PCM sample to 4-bit ADPCM
    uint8_t encodeSample(int16_t sample);
    
public:
    ADPCMEncoder();
    
    // Reset encoder state
    void reset();
    
    // Encode PCM buffer to ADPCM
    // Input: pcmBuffer (16-bit samples), sampleCount
    // Output: adpcmBuffer (4-bit samples packed into bytes)
    // Returns: number of bytes written to adpcmBuffer
    size_t encode(const int16_t* pcmBuffer, size_t sampleCount, uint8_t* adpcmBuffer);
    
    // Encode PCM buffer in-place (for streaming)
    // Input/Output: buffer contains PCM input, will contain ADPCM output
    // Returns: new size of buffer after compression
    size_t encodeInPlace(uint8_t* buffer, size_t pcmBytes);
    
    // Get compression ratio
    static constexpr float getCompressionRatio() { return 4.0f; }
    
    // Get state for debugging
    void getState(int16_t& prevVal, int8_t& idx) const {
        prevVal = valprev;
        idx = index;
    }
};

class ADPCMDecoder {
private:
    int16_t valprev;     // Previous output value
    int8_t index;        // Index into stepsize table
    
    // ADPCM step size table (same as encoder)
    static const int16_t stepsizeTable[89];
    
    // Index adjustment table (same as encoder)
    static const int8_t indexTable[16];
    
    // Decode a single 4-bit ADPCM sample to 16-bit PCM
    int16_t decodeSample(uint8_t adpcmSample);
    
public:
    ADPCMDecoder();
    
    // Reset decoder state
    void reset();
    
    // Decode ADPCM buffer to PCM
    // Input: adpcmBuffer (4-bit samples packed into bytes), adpcmBytes
    // Output: pcmBuffer (16-bit samples)
    // Returns: number of samples written to pcmBuffer
    size_t decode(const uint8_t* adpcmBuffer, size_t adpcmBytes, int16_t* pcmBuffer);
    
    // Decode ADPCM buffer in-place (for streaming)
    // Input: buffer contains ADPCM data
    // Output: buffer will contain PCM data (buffer must be large enough)
    // Returns: new size of buffer after decompression
    size_t decodeInPlace(uint8_t* buffer, size_t adpcmBytes);
    
    // Get expansion ratio
    static constexpr float getExpansionRatio() { return 4.0f; }
    
    // Get state for debugging
    void getState(int16_t& prevVal, int8_t& idx) const {
        prevVal = valprev;
        idx = index;
    }
};

// Utility functions for ADPCM processing
namespace ADPCM {
    // Calculate compressed size for given PCM sample count
    inline size_t getCompressedSize(size_t pcmSamples) {
        return (pcmSamples + 1) / 2; // 2 samples per byte
    }
    
    // Calculate decompressed size for given ADPCM byte count
    inline size_t getDecompressedSize(size_t adpcmBytes) {
        return adpcmBytes * 2; // 2 samples per byte
    }
    
    // Calculate PCM byte count from sample count
    inline size_t pcmSamplesToBytes(size_t samples) {
        return samples * sizeof(int16_t);
    }
    
    // Calculate sample count from PCM byte count
    inline size_t pcmBytesToSamples(size_t bytes) {
        return bytes / sizeof(int16_t);
    }
}

// Stream wrapper for ADPCM encoding (compatible with AudioTools)
class ADPCMEncoderStream : public Print {
private:
    ADPCMEncoder encoder;
    Print* output;
    uint8_t* tempBuffer;
    size_t tempBufferSize;
    size_t tempBufferUsed;
    
public:
    ADPCMEncoderStream(Print* outputStream, size_t bufferSize = 1024);
    ~ADPCMEncoderStream();
    
    // Print interface implementation
    virtual size_t write(uint8_t b) override;
    virtual size_t write(const uint8_t *buffer, size_t size) override;
    
    // Flush any remaining data
    void flush();
    
    // Reset encoder state
    void reset();
};

#endif // ADPCM_H
