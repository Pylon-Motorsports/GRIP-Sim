#include "VulkanContext.h"
#include "Renderer.h"
#include "Vehicle.h"
#include "VehiclePhysics.h"
#include "Input.h"
#include "Scenario.h"
#include "TireTrail.h"
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
    TireTrails tireTrails;

    // Build the playground
    Playground playground = createPlayground();
    physics.setBumps(playground.bumps.empty() ? nullptr : &playground.bumps);

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

            // Reset vehicle: R key
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r) {
                physics.reset();
                tireTrails.clear();
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
        if (keys[SDL_SCANCODE_W] || keys[SDL_SCANCODE_UP])    input.throttle = 1.f;
        if (keys[SDL_SCANCODE_S] || keys[SDL_SCANCODE_DOWN])  input.brake    = 1.f;
        if (keys[SDL_SCANCODE_A] || keys[SDL_SCANCODE_LEFT])  input.steer    = -1.f;
        if (keys[SDL_SCANCODE_D] || keys[SDL_SCANCODE_RIGHT]) input.steer    =  1.f;

        if (controller) {
            float rt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.f;
            float lt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT)  / 32767.f;
            float lx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX)        / 32767.f;
            if (rt > 0.05f) input.throttle = rt;
            if (lt > 0.05f) input.brake    = lt;
            if (std::abs(lx) > 0.05f) input.steer = lx;
            input.clutchIn = SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_RIGHTSHOULDER);
        }

        // Keyboard clutch: C key
        if (keys[SDL_SCANCODE_C]) input.clutchIn = true;

        // Fixed-timestep physics
        while (accumulator >= PHYSICS_DT) {
            physics.update(PHYSICS_DT, input);
            accumulator -= PHYSICS_DT;
        }

        physics.fillVehicle(vehicle);
        tireTrails.update(vehicle);

        // Build trail geometry
        std::vector<Vertex> trailVerts;
        std::vector<uint32_t> trailIdx;
        tireTrails.buildGeometry(trailVerts, trailIdx);
        TrailGeometry trailGeo;
        trailGeo.verts    = trailVerts.data();
        trailGeo.vertCount = (uint32_t)trailVerts.size();
        trailGeo.indices  = trailIdx.data();
        trailGeo.idxCount = (uint32_t)trailIdx.size();

        // Build HUD data
        HudData hud;
        hud.speedKmh       = std::abs(physics.getFrontWheelSpeed()) * 3.6f;
        hud.rpm            = physics.getEngineRpm();
        hud.rpmLimit       = physics.getEngineRpmLimit();
        hud.gear           = physics.getGear();

        VkCommandBuffer cmd = ctx.beginFrame();
        if (cmd) {
            renderer.draw(cmd, ctx.swapExtent.width, ctx.swapExtent.height,
                          vehicle, hud, playground, &trailGeo);
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
