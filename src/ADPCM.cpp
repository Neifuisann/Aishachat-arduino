#include "ADPCM.h"
#include <algorithm>
#include "esp_heap_caps.h"  // For ESP32 heap management

// ADPCM step size table
const int16_t ADPCMEncoder::stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

// Index adjustment table
const int8_t ADPCMEncoder::indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

// Same tables for decoder
const int16_t ADPCMDecoder::stepsizeTable[89] = {
    7, 8, 9, 10, 11, 12, 13, 14, 16, 17,
    19, 21, 23, 25, 28, 31, 34, 37, 41, 45,
    50, 55, 60, 66, 73, 80, 88, 97, 107, 118,
    130, 143, 157, 173, 190, 209, 230, 253, 279, 307,
    337, 371, 408, 449, 494, 544, 598, 658, 724, 796,
    876, 963, 1060, 1166, 1282, 1411, 1552, 1707, 1878, 2066,
    2272, 2499, 2749, 3024, 3327, 3660, 4026, 4428, 4871, 5358,
    5894, 6484, 7132, 7845, 8630, 9493, 10442, 11487, 12635, 13899,
    15289, 16818, 18500, 20350, 22385, 24623, 27086, 29794, 32767
};

const int8_t ADPCMDecoder::indexTable[16] = {
    -1, -1, -1, -1, 2, 4, 6, 8,
    -1, -1, -1, -1, 2, 4, 6, 8
};

// ============================================================================
// ADPCMEncoder Implementation
// ============================================================================

ADPCMEncoder::ADPCMEncoder() {
    reset();
}

void ADPCMEncoder::reset() {
    valprev = 0;
    index = 0;
}

uint8_t ADPCMEncoder::encodeSample(int16_t sample) {
    int16_t step = stepsizeTable[index];
    int16_t diff = sample - valprev;
    
    uint8_t code = 0;
    
    // Set sign bit
    if (diff < 0) {
        code = 8;
        diff = -diff;
    }
    
    // Quantize the difference
    if (diff >= step) {
        code |= 4;
        diff -= step;
    }
    step >>= 1;
    
    if (diff >= step) {
        code |= 2;
        diff -= step;
    }
    step >>= 1;
    
    if (diff >= step) {
        code |= 1;
    }
    
    // Reconstruct the signal
    int16_t diffq = stepsizeTable[index] >> 3;
    if (code & 4) diffq += stepsizeTable[index];
    if (code & 2) diffq += stepsizeTable[index] >> 1;
    if (code & 1) diffq += stepsizeTable[index] >> 2;
    
    if (code & 8) {
        valprev -= diffq;
    } else {
        valprev += diffq;
    }
    
    // Clamp to 16-bit range
    if (valprev > 32767) valprev = 32767;
    else if (valprev < -32768) valprev = -32768;
    
    // Update index
    index += indexTable[code];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;
    
    return code & 0x0F;
}

size_t ADPCMEncoder::encode(const int16_t* pcmBuffer, size_t sampleCount, uint8_t* adpcmBuffer) {
    size_t adpcmBytes = 0;
    
    for (size_t i = 0; i < sampleCount; i += 2) {
        uint8_t byte = 0;
        
        // Encode first sample (lower 4 bits)
        byte = encodeSample(pcmBuffer[i]);
        
        // Encode second sample (upper 4 bits) if available
        if (i + 1 < sampleCount) {
            byte |= (encodeSample(pcmBuffer[i + 1]) << 4);
        }
        
        adpcmBuffer[adpcmBytes++] = byte;
    }
    
    return adpcmBytes;
}

size_t ADPCMEncoder::encodeInPlace(uint8_t* buffer, size_t pcmBytes) {
    size_t sampleCount = pcmBytes / sizeof(int16_t);
    int16_t* pcmBuffer = (int16_t*)buffer;
    
    // Encode in-place (working backwards to avoid overwriting)
    size_t adpcmBytes = ADPCM::getCompressedSize(sampleCount);
    
    for (size_t i = 0; i < sampleCount; i += 2) {
        uint8_t byte = 0;
        
        // Encode first sample (lower 4 bits)
        byte = encodeSample(pcmBuffer[i]);
        
        // Encode second sample (upper 4 bits) if available
        if (i + 1 < sampleCount) {
            byte |= (encodeSample(pcmBuffer[i + 1]) << 4);
        }
        
        buffer[i / 2] = byte;
    }
    
    return adpcmBytes;
}

// ============================================================================
// ADPCMDecoder Implementation
// ============================================================================

ADPCMDecoder::ADPCMDecoder() {
    reset();
}

void ADPCMDecoder::reset() {
    valprev = 0;
    index = 0;
}

int16_t ADPCMDecoder::decodeSample(uint8_t adpcmSample) {
    int16_t step = stepsizeTable[index];
    int16_t diffq = step >> 3;
    
    if (adpcmSample & 4) diffq += step;
    if (adpcmSample & 2) diffq += step >> 1;
    if (adpcmSample & 1) diffq += step >> 2;
    
    if (adpcmSample & 8) {
        valprev -= diffq;
    } else {
        valprev += diffq;
    }
    
    // Clamp to 16-bit range
    if (valprev > 32767) valprev = 32767;
    else if (valprev < -32768) valprev = -32768;
    
    // Update index
    index += indexTable[adpcmSample];
    if (index < 0) index = 0;
    else if (index > 88) index = 88;
    
    return valprev;
}

size_t ADPCMDecoder::decode(const uint8_t* adpcmBuffer, size_t adpcmBytes, int16_t* pcmBuffer) {
    size_t sampleCount = 0;
    
    for (size_t i = 0; i < adpcmBytes; i++) {
        uint8_t byte = adpcmBuffer[i];
        
        // Decode first sample (lower 4 bits)
        pcmBuffer[sampleCount++] = decodeSample(byte & 0x0F);
        
        // Decode second sample (upper 4 bits)
        pcmBuffer[sampleCount++] = decodeSample((byte >> 4) & 0x0F);
    }
    
    return sampleCount;
}

size_t ADPCMDecoder::decodeInPlace(uint8_t* buffer, size_t adpcmBytes) {
    // Need temporary storage since we're expanding
    int16_t* tempPcm = (int16_t*)heap_caps_malloc(adpcmBytes * 2 * sizeof(int16_t), MALLOC_CAP_DEFAULT);
    if (!tempPcm) {
        Serial.printf("ADPCM: Failed to allocate %d bytes for decode buffer!\n", adpcmBytes * 2 * sizeof(int16_t));
        return 0;
    }

    size_t sampleCount = decode(buffer, adpcmBytes, tempPcm);

    // Copy back to original buffer
    memcpy(buffer, tempPcm, sampleCount * sizeof(int16_t));
    heap_caps_free(tempPcm);

    return sampleCount * sizeof(int16_t);
}

// ============================================================================
// ADPCMEncoderStream Implementation
// ============================================================================

ADPCMEncoderStream::ADPCMEncoderStream(Print* outputStream, size_t bufferSize)
    : output(outputStream), tempBufferSize(bufferSize), tempBufferUsed(0) {
    // Use ESP32 heap_caps_malloc for better memory management
    tempBuffer = (uint8_t*)heap_caps_malloc(bufferSize, MALLOC_CAP_DEFAULT);
    if (!tempBuffer) {
        Serial.printf("ADPCM: Failed to allocate %d bytes for encoder buffer!\n", bufferSize);
        tempBufferSize = 0;
    }
    encoder.reset();
}

ADPCMEncoderStream::~ADPCMEncoderStream() {
    flush();
    if (tempBuffer) {
        heap_caps_free(tempBuffer);
        tempBuffer = nullptr;
    }
}

size_t ADPCMEncoderStream::write(uint8_t b) {
    return write(&b, 1);
}

size_t ADPCMEncoderStream::write(const uint8_t *buffer, size_t size) {
    if (!tempBuffer || !output || tempBufferSize == 0) return 0;
    
    size_t written = 0;
    
    while (size > 0) {
        size_t canCopy = std::min(size, tempBufferSize - tempBufferUsed);
        memcpy(tempBuffer + tempBufferUsed, buffer + written, canCopy);
        tempBufferUsed += canCopy;
        written += canCopy;
        size -= canCopy;
        
        // Process when buffer is full or we have enough for ADPCM encoding
        if (tempBufferUsed >= tempBufferSize || 
            (tempBufferUsed >= sizeof(int16_t) * 2 && size == 0)) {
            
            // Ensure we have even number of samples for ADPCM
            size_t samplesToProcess = (tempBufferUsed / sizeof(int16_t)) & ~1;
            size_t bytesToProcess = samplesToProcess * sizeof(int16_t);
            
            if (bytesToProcess > 0) {
                size_t compressedSize = encoder.encodeInPlace(tempBuffer, bytesToProcess);
                output->write(tempBuffer, compressedSize);
                
                // Move remaining bytes to beginning
                size_t remaining = tempBufferUsed - bytesToProcess;
                if (remaining > 0) {
                    memmove(tempBuffer, tempBuffer + bytesToProcess, remaining);
                }
                tempBufferUsed = remaining;
            }
        }
    }
    
    return written;
}

void ADPCMEncoderStream::flush() {
    if (tempBufferUsed > 0 && tempBuffer && output) {
        // Pad with zero if odd number of samples
        if ((tempBufferUsed / sizeof(int16_t)) & 1) {
            if (tempBufferUsed + sizeof(int16_t) <= tempBufferSize) {
                *((int16_t*)(tempBuffer + tempBufferUsed)) = 0;
                tempBufferUsed += sizeof(int16_t);
            }
        }
        
        if (tempBufferUsed >= sizeof(int16_t)) {
            size_t compressedSize = encoder.encodeInPlace(tempBuffer, tempBufferUsed);
            output->write(tempBuffer, compressedSize);
        }
        tempBufferUsed = 0;
    }
}

void ADPCMEncoderStream::reset() {
    encoder.reset();
    tempBufferUsed = 0;
}
