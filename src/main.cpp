#include "VulkanContext.h"
#include "Renderer.h"
#include "Vehicle.h"
#include "VehiclePhysics.h"
#include <SDL2/SDL.h>
#include <cstdio>

int main(int /*argc*/, char** /*argv*/)
{
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
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

    constexpr float PHYSICS_DT = 1.f / 120.f;
    float accumulator = 0.f;
    Uint64 lastTick = SDL_GetPerformanceCounter();
    Uint64 freq     = SDL_GetPerformanceFrequency();

    bool running = true;
    while (running) {
        Uint64 now = SDL_GetPerformanceCounter();
        float frameDt = (float)(now - lastTick) / (float)freq;
        lastTick = now;

        // Cap frame dt to prevent spiral of death
        if (frameDt > 0.1f) frameDt = 0.1f;
        accumulator += frameDt;

        SDL_Event e;
        while (SDL_PollEvent(&e)) {
            if (e.type == SDL_QUIT) running = false;
            if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE) running = false;
        }

        // Fixed-timestep physics
        while (accumulator >= PHYSICS_DT) {
            physics.update(PHYSICS_DT);
            accumulator -= PHYSICS_DT;
        }

        physics.fillVehicle(vehicle);

        VkCommandBuffer cmd = ctx.beginFrame();
        if (cmd) {
            renderer.draw(cmd, ctx.swapExtent.width, ctx.swapExtent.height, vehicle);
            ctx.endFrame();
        }
    }

    vkDeviceWaitIdle(ctx.device);
    renderer.shutdown(ctx.device);
    ctx.shutdown();
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
