#include "VulkanContext.h"
#include "Renderer.h"
#include "Vehicle.h"
#include "VehiclePhysics.h"
#include "Input.h"
#include "Scenario.h"
#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>

int main(int /*argc*/, char** /*argv*/)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_GAMECONTROLLER) != 0) {
        std::fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "GRIP-Sim",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1600, 900,
        SDL_WINDOW_VULKAN | SDL_WINDOW_SHOWN);

    if (!window) {
        std::fprintf(stderr, "SDL_CreateWindow: %s\n", SDL_GetError());
        return 1;
    }

    VulkanContext ctx;
    if (!ctx.init(window)) { std::fprintf(stderr, "Vulkan init failed\n"); return 1; }

    Renderer renderer;
    if (!renderer.init(ctx)) { std::fprintf(stderr, "Renderer init failed\n"); return 1; }

    VehiclePhysics physics;
    physics.init();

    Vehicle vehicle;

    // Scenarios
    auto scenarios = createScenarios();
    int activeScenario = 0;

    // Collect scenario name pointers for HUD
    std::vector<const char*> scenarioNames;
    for (auto& s : scenarios) scenarioNames.push_back(s.name);

    auto switchScenario = [&](int idx) {
        if (idx < 0 || idx >= (int)scenarios.size() || idx == activeScenario) return;
        activeScenario = idx;
        physics.setBumps(scenarios[idx].bumps.empty() ? nullptr : &scenarios[idx].bumps);
        physics.reset();
        std::printf("Scenario: %s\n", scenarios[idx].name);
    };

    // Start with flat
    physics.setBumps(nullptr);

    // Try to open a game controller
    SDL_GameController* controller = nullptr;
    for (int i = 0; i < SDL_NumJoysticks(); ++i) {
        if (SDL_IsGameController(i)) {
            controller = SDL_GameControllerOpen(i);
            if (controller) {
                std::printf("Controller: %s\n", SDL_GameControllerName(controller));
                break;
            }
        }
    }

    constexpr float PHYSICS_DT = 1.f / 120.f;
    float accumulator = 0.f;
    Uint64 lastTick = SDL_GetPerformanceCounter();
    Uint64 freq     = SDL_GetPerformanceFrequency();

    bool running = true;
    while (running) {
        Uint64 now = SDL_GetPerformanceCounter();
        float frameDt = (float)(now - lastTick) / (float)freq;
        lastTick = now;

        if (frameDt > 0.1f) frameDt = 0.1f;
        accumulator += frameDt;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = false;

            // Scenario switching: keys 1, 2, 3
            if (e.type == SDL_KEYDOWN) {
                if (e.key.keysym.sym == SDLK_1) switchScenario(0);
                if (e.key.keysym.sym == SDLK_2) switchScenario(1);
                if (e.key.keysym.sym == SDLK_3) switchScenario(2);
            }

            // Mouse click on scenario buttons
            if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                int hit = renderer.hitTestButton(
                    e.button.x, e.button.y,
                    ctx.swapExtent.width, ctx.swapExtent.height,
                    (int)scenarios.size());
                if (hit >= 0) switchScenario(hit);
            }

            if (e.type == SDL_CONTROLLERDEVICEADDED && !controller) {
                controller = SDL_GameControllerOpen(e.cdevice.which);
                if (controller)
                    std::printf("Controller connected: %s\n", SDL_GameControllerName(controller));
            }
            if (e.type == SDL_CONTROLLERDEVICEREMOVED && controller) {
                SDL_GameControllerClose(controller);
                controller = nullptr;
                std::printf("Controller disconnected\n");
            }
        }

        // --- Build input state ---
        InputState input{};

        const Uint8* keys = SDL_GetKeyboardState(nullptr);
        if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP])   input.throttle = 1.f;
        if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN]) input.brake    = 1.f;

        if (controller) {
            float rt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.f;
            float lt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT)  / 32767.f;
            if (rt > 0.05f) input.throttle = rt;
            if (lt > 0.05f) input.brake    = lt;
        }

        // Fixed-timestep physics
        while (accumulator >= PHYSICS_DT) {
            physics.update(PHYSICS_DT, input);
            accumulator -= PHYSICS_DT;
        }

        physics.fillVehicle(vehicle);

        // Build HUD data
        HudData hud;
        hud.activeScenario = activeScenario;
        hud.numScenarios   = (int)scenarios.size();
        hud.scenarioNames  = scenarioNames.data();
        hud.speedKmh       = std::abs(physics.getFrontWheelSpeed()) * 3.6f;
        hud.rpm            = physics.getEngineRpm();
        hud.rpmLimit       = physics.getEngineRpmLimit();

        VkCommandBuffer cmd = ctx.beginFrame();
        if (cmd) {
            renderer.draw(cmd, ctx.swapExtent.width, ctx.swapExtent.height,
                          vehicle, hud, scenarios[activeScenario].bumps);
            ctx.endFrame();
        }
    }

    vkDeviceWaitIdle(ctx.device);
    renderer.shutdown(ctx.device);
    ctx.shutdown();
    if (controller) SDL_GameControllerClose(controller);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
