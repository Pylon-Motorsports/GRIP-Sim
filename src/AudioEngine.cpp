#include "AudioEngine.hpp"
#include <cstdio>
#include <cstring>
#include <algorithm>

bool AudioEngine::init()
{
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        std::fprintf(stderr, "SDL_InitSubSystem(AUDIO): %s\n", SDL_GetError());
        return false;
    }

    SDL_AudioSpec want{};
    want.freq     = 44100;
    want.format   = AUDIO_F32SYS;
    want.channels = 1;
    want.samples  = 512;  // ~11.6 ms latency at 44100 Hz
    want.callback = audioCallback;
    want.userdata = this;

    SDL_AudioSpec got{};
    device_ = SDL_OpenAudioDevice(nullptr, 0, &want, &got, 0);
    if (device_ == 0) {
        std::fprintf(stderr, "SDL_OpenAudioDevice: %s\n", SDL_GetError());
        return false;
    }

    sampleRate_ = got.freq;
    SDL_PauseAudioDevice(device_, 0);  // start playback
    std::printf("Audio: %d Hz, %d ch, %d sample buffer\n", got.freq, got.channels, got.samples);
    return true;
}

void AudioEngine::shutdown()
{
    if (device_) {
        SDL_CloseAudioDevice(device_);
        device_ = 0;
    }
}

void AudioEngine::setEngineRpm(float rpm, float rpmLimit, float throttle)
{
    // Map RPM to frequency: idle (~1200 RPM) = ~40 Hz, redline (~6500) = ~220 Hz
    // This gives a satisfying pitch sweep across the rev range.
    float rpmNorm = (rpm - 800.f) / (rpmLimit - 800.f);
    rpmNorm = std::clamp(rpmNorm, 0.f, 1.f);

    float freq = 40.f + rpmNorm * 180.f;
    engineFreq_.store(freq, std::memory_order_relaxed);

    // Volume: base idle rumble + louder with throttle
    float vol = 0.08f + throttle * 0.15f;
    engineVol_.store(vol, std::memory_order_relaxed);
}

void AudioEngine::setTireSlip(float maxSlipRatio, float maxSlipAngleDeg)
{
    // Squeal kicks in above moderate slip levels
    float slipFactor = std::max(
        std::clamp((std::abs(maxSlipRatio) - 0.08f) / 0.3f, 0.f, 1.f),
        std::clamp((maxSlipAngleDeg - 5.f) / 20.f, 0.f, 1.f)
    );
    float vol = slipFactor * 0.03f;  // quietened (was 0.12)
    squealVol_.store(vol, std::memory_order_relaxed);
}

void AudioEngine::audioCallback(void* userdata, Uint8* stream, int len)
{
    auto* self = static_cast<AudioEngine*>(userdata);
    int frames = len / sizeof(float);
    self->generate(reinterpret_cast<float*>(stream), frames);
}

void AudioEngine::generate(float* out, int frames)
{
    float freq     = engineFreq_.load(std::memory_order_relaxed);
    float engVol   = engineVol_.load(std::memory_order_relaxed);
    float sqVol    = squealVol_.load(std::memory_order_relaxed);
    float invRate  = 1.f / (float)sampleRate_;
    constexpr float TAU = 6.2831853f;

    // Biquad bandpass coefficients for tire squeal.
    // Center ~1800 Hz, Q ~2.5 — gives a broad screech band (not tonal).
    // Computed once per buffer (coefficients don't change within a buffer).
    constexpr float centerFreq = 1800.f;
    constexpr float Q = 2.5f;
    float w0 = TAU * centerFreq * invRate;
    float alpha = std::sin(w0) / (2.f * Q);
    float b0 =  alpha;
    float b1 =  0.f;
    float b2 = -alpha;
    float a0 =  1.f + alpha;
    float a1 = -2.f * std::cos(w0);
    float a2 =  1.f - alpha;
    // Normalize
    b0 /= a0; b1 /= a0; b2 /= a0;
    a1 /= a0; a2 /= a0;

    for (int i = 0; i < frames; ++i) {
        // Engine: fundamental + 2nd harmonic
        float eng = std::sin(enginePhase_ * TAU) * 0.6f
                  + std::sin(enginePhase2_ * TAU) * 0.3f;

        enginePhase_  += freq * invRate;
        enginePhase2_ += freq * 2.f * invRate;
        if (enginePhase_  > 1.f) enginePhase_  -= 1.f;
        if (enginePhase2_ > 1.f) enginePhase2_ -= 1.f;

        // Tire squeal: white noise → biquad bandpass → continuous screech.
        // Band-passed noise sounds like rubber-on-tarmac stick-slip resonance
        // without the metallic/tonal character of pure oscillators.
        float x = noise();
        float squeal = b0 * x + b1 * bqX1_ + b2 * bqX2_
                     - a1 * bqY1_ - a2 * bqY2_;
        bqX2_ = bqX1_; bqX1_ = x;
        bqY2_ = bqY1_; bqY1_ = squeal;

        out[i] = eng * engVol + squeal * sqVol;
    }
}
