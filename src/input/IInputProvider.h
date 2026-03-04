#pragma once
#include "InputFrame.h"

/// Module interface for player input.
/// Swap implementations for different controller types, keyboard, replay, AI, etc.
class IInputProvider {
public:
    virtual ~IInputProvider() = default;

    /// One-time initialization. Returns false on failure.
    virtual bool initialize() = 0;

    /// Poll OS/hardware events. Must be called once per frame before reading frame().
    virtual void poll() = 0;

    /// Returns the most recently polled input frame.
    [[nodiscard]] virtual const InputFrame& frame() const = 0;

    /// Returns true if the user has requested application exit.
    [[nodiscard]] virtual bool shouldQuit() const = 0;

    /// Clean up resources.
    virtual void shutdown() = 0;
};
