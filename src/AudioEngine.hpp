#pragma once
#include <SDL2/SDL.h>
#include <atomic>
#include <cmath>

// Real-time synthesized audio: engine tone (RPM-tracking) + tire squeal (slip-based noise).
// All parameters are set from the game thread via atomics; the SDL audio callback
// reads them lock-free on the audio thread.
class AudioEngine {
public:
    bool init();
    void shutdown();

    // Call each frame from game thread to update audio parameters.
    void setEngineRpm(float rpm, float rpmLimit, float throttle);
    void setTireSlip(float maxSlipRatio, float maxSlipAngleDeg);

    // Trigger a one-shot impact sound. intensity in [0,1] scales volume.
    void triggerCollision(float intensity);

private:
    SDL_AudioDeviceID device_ = 0;
    int sampleRate_ = 0;

    // Phase accumulators (audio thread only)
    float enginePhase_   = 0.f;
    float enginePhase2_  = 0.f;  // second harmonic

    // Tire squeal: band-passed noise through biquad filter
    // Simulates stick-slip rubber resonance (continuous screech, not tonal)
    uint32_t noiseState_ = 0x12345678;  // xorshift PRNG state
    float bqX1_ = 0.f, bqX2_ = 0.f;    // biquad input history
    float bqY1_ = 0.f, bqY2_ = 0.f;    // biquad output history

    // Collision impact: decaying noise burst
    float impactEnergy_ = 0.f;               // audio thread — remaining energy
    float impactLpState_ = 0.f;              // audio thread — lowpass filter state

    // Shared state (written by game thread, read by audio thread)
    std::atomic<float> engineFreq_{80.f};    // Hz (maps from RPM)
    std::atomic<float> engineVol_{0.f};      // [0, 1]
    std::atomic<float> squealVol_{0.f};      // [0, 1]
    std::atomic<float> impactTrigger_{0.f};  // >0 triggers new impact

    static void audioCallback(void* userdata, Uint8* stream, int len);
    void generate(float* out, int frames);

    // Fast xorshift noise [-1, 1]
    float noise() {
        noiseState_ ^= noiseState_ << 13;
        noiseState_ ^= noiseState_ >> 17;
        noiseState_ ^= noiseState_ << 5;
        return (float)(int32_t)noiseState_ / (float)0x7FFFFFFF;
    }
};
