#pragma once
#include "IPaceNoteReader.h"

/// IPaceNoteReader implementation that prints pace notes to stdout.
/// Useful as a fallback and for non-Windows platforms.
class ConsolePaceNoteReader : public IPaceNoteReader {
public:
    bool initialize() override { return true; }
    void speak(const PaceNote& note) override;
    void shutdown() override {}
};
