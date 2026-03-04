#pragma once
#include "PaceNote.h"

/// Module interface for pace note delivery.
/// Swap implementations for different TTS engines, audio files, display-only, etc.
class IPaceNoteReader {
public:
    virtual ~IPaceNoteReader() = default;

    /// One-time initialization. Returns false on failure.
    virtual bool initialize() = 0;

    /// Deliver the given pace note (speak it, display it, etc.).
    virtual void speak(const PaceNote& note) = 0;

    /// Clean shutdown.
    virtual void shutdown() = 0;
};
