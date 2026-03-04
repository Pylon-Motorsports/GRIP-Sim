#pragma once
#include "IInputProvider.h"
#include <SDL2/SDL.h>

/// IInputProvider implementation using SDL2's game controller API.
/// Supports Xbox controllers (and any SDL-compatible gamepad).
///
/// Axis mapping:
///   Left stick X              → steer
///   Right trigger (axis 5)    → throttle
///   Left trigger  (axis 4)    → brake
///   Right bumper  (SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) → handbrake
///   Start                     → reset car
///   Back/Select               → quit
class SdlInputProvider : public IInputProvider {
public:
    bool initialize() override;
    void poll() override;
    [[nodiscard]] const InputFrame& frame() const override { return frame_; }
    [[nodiscard]] bool shouldQuit() const override { return frame_.quit; }
    void shutdown() override;

private:
    InputFrame        frame_;
    SDL_GameController* controller_ { nullptr };

    static float normalizeAxis(int16_t raw, float deadzone = 0.08f);
    static float normalizeTrigger(int16_t raw);
};
