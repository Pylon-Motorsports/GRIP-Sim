#include "SdlInputProvider.h"
#include "core/Logging.h"
#include <cstring>

bool SdlInputProvider::initialize()
{
    if (SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) < 0) {
        LOG_WARN("SDL_INIT_GAMECONTROLLER failed: %s", SDL_GetError());
        return true;  // Non-fatal: fall back to keyboard-only
    }

    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller_ = SDL_GameControllerOpen(i);
            if (controller_) {
                LOG_INFO("Controller: %s", SDL_GameControllerName(controller_));
                break;
            }
        }
    }
    if (!controller_) LOG_INFO("No controller found — keyboard fallback active");
    return true;
}

float SdlInputProvider::normalizeAxis(int16_t raw, float deadzone)
{
    float v = raw / 32767.f;
    if (v >  deadzone) return  (v - deadzone) / (1.f - deadzone);
    if (v < -deadzone) return -((-v - deadzone) / (1.f - deadzone));
    return 0.f;
}

float SdlInputProvider::normalizeTrigger(int16_t raw)
{
    // Triggers: 0..32767
    return raw / 32767.f;
}

void SdlInputProvider::poll()
{
    frame_.resetCar = false;

    SDL_Event event;
    while (SDL_PollEvent(&event)) {
        switch (event.type) {
            case SDL_QUIT:
                frame_.quit = true;
                break;
            case SDL_KEYDOWN:
                switch (event.key.keysym.sym) {
                    case SDLK_ESCAPE: frame_.quit     = true; break;
                    case SDLK_r:      frame_.resetCar = true; break;
                    default: break;
                }
                break;
            case SDL_CONTROLLERDEVICEADDED:
                if (!controller_) {
                    controller_ = SDL_GameControllerOpen(event.cdevice.which);
                    LOG_INFO("Controller connected: %s", SDL_GameControllerName(controller_));
                }
                break;
            case SDL_CONTROLLERDEVICEREMOVED:
                LOG_INFO("Controller disconnected");
                if (controller_) {
                    SDL_GameControllerClose(controller_);
                    controller_ = nullptr;
                }
                break;
            default: break;
        }
    }

    if (controller_) {
        // Steer: left stick X axis with power curve for fine control near center.
        // Exponent 2.0 = quadratic: half-stick gives 25% steering, full-stick = full lock.
        int16_t steerRaw = SDL_GameControllerGetAxis(controller_,
            SDL_CONTROLLER_AXIS_LEFTX);
        float steerLinear = normalizeAxis(steerRaw);
        float sign = (steerLinear >= 0.f) ? 1.f : -1.f;
        frame_.steer = sign * steerLinear * steerLinear;  // |x|^2 preserving sign

        // Throttle: right trigger
        int16_t throttleRaw = SDL_GameControllerGetAxis(controller_,
            SDL_CONTROLLER_AXIS_TRIGGERRIGHT);
        frame_.throttle = normalizeTrigger(throttleRaw);

        // Brake: left trigger
        int16_t brakeRaw = SDL_GameControllerGetAxis(controller_,
            SDL_CONTROLLER_AXIS_TRIGGERLEFT);
        frame_.brake = normalizeTrigger(brakeRaw);

        // Handbrake: right bumper
        frame_.handbrake = SDL_GameControllerGetButton(controller_,
            SDL_CONTROLLER_BUTTON_RIGHTSHOULDER) ? 1.f : 0.f;

        // Reset: Start button
        if (SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_START))
            frame_.resetCar = true;

        // Quit: Back/Select button
        if (SDL_GameControllerGetButton(controller_, SDL_CONTROLLER_BUTTON_BACK))
            frame_.quit = true;
    } else {
        // Keyboard fallback
        const uint8_t* keys = SDL_GetKeyboardState(nullptr);
        frame_.steer    = (keys[SDL_SCANCODE_LEFT] ? -1.f : 0.f)
                        + (keys[SDL_SCANCODE_RIGHT] ?  1.f : 0.f);
        frame_.throttle = keys[SDL_SCANCODE_UP]    ? 1.f : 0.f;
        frame_.brake    = keys[SDL_SCANCODE_DOWN]  ? 1.f : 0.f;
    }
}

void SdlInputProvider::shutdown()
{
    if (controller_) {
        SDL_GameControllerClose(controller_);
        controller_ = nullptr;
    }
}
