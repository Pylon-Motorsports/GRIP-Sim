#pragma once
#ifdef _WIN32
#include "IPaceNoteReader.h"

// Forward declare COM interface to avoid including heavy Windows headers here
struct ISpVoice;

/// IPaceNoteReader implementation using Windows SAPI (Speech API).
/// Reads pace notes aloud using the system default TTS voice.
/// Speech is asynchronous (non-blocking) — the call returns immediately.
class WindowsSapiReader : public IPaceNoteReader {
public:
    bool initialize() override;
    void speak(const PaceNote& note) override;
    void shutdown() override;

private:
    ISpVoice* voice_ { nullptr };
};

#endif // _WIN32
