#include "VulkanContext.hpp"
#include "Renderer.hpp"
#include "Vehicle.hpp"
#include "VehiclePhysics.hpp"
#include "Scenario.hpp"
#include "TireTrail.hpp"
#include "AudioEngine.hpp"
#include "Terrain.hpp"
#include <SDL2/SDL.h>
#include <cstdio>
#include <cmath>
#include <algorithm>

// ---------------------------------------------------------------------------
// Terrain interaction helpers
// ---------------------------------------------------------------------------

struct WheelInfo {
    Subframe* subframe;
    SuspensionCorner* corner;
};

static glm::vec3 wheelLocalPos(const WheelInfo& w) {
    return w.subframe->attachmentPoint()
         + w.corner->attachmentPoint()
         + w.corner->tireOffset;
}

static glm::vec3 mountLocalPos(const WheelInfo& w) {
    return w.subframe->attachmentPoint()
         + w.corner->attachmentPoint();
}

// Set spring compressions from terrain height at each wheel.
// Returns per-wheel ground height for ground clamp logic.
static void updateSprings(VehiclePhysics& physics, const Terrain& terrain,
                           float prevComp[4], float dt)
{
    const auto& st = physics.state();

    WheelInfo wheels[4] = {
        { physics.frontSubframe(), physics.frontSubframe()->left()  },
        { physics.frontSubframe(), physics.frontSubframe()->right() },
        { physics.rearSubframe(),  physics.rearSubframe()->left()   },
        { physics.rearSubframe(),  physics.rearSubframe()->right()  },
    };

    for (int i = 0; i < 4; ++i) {
        glm::vec3 local = wheelLocalPos(wheels[i]);
        glm::vec3 world = st.position + st.bodyRotation * local;

        float groundH = terrain.heightAt(world.x, world.z);
        float tireR   = wheels[i].corner->tire()->radius;

        // Compression: how much ground pushes tire up from rest position
        float comp = (groundH + tireR) - world.y;
        comp = std::max(0.f, comp);

        // Compression velocity from finite difference
        float compVel = (dt > 0.f) ? (comp - prevComp[i]) / dt : 0.f;
        prevComp[i] = comp;

        wheels[i].corner->spring()->setCompression(comp, compVel);
    }
}

// Check body collider corners against terrain, build collision contacts.
static std::vector<CollisionContact> checkBodyCollisions(
    const VehiclePhysics& physics, const Terrain& terrain)
{
    std::vector<CollisionContact> contacts;
    auto* body = const_cast<VehiclePhysics&>(physics).body();
    if (!body) return contacts;

    const auto& st = physics.state();
    auto corners = body->colliderCorners();

    for (auto& local : corners) {
        // Body-local to world: collider is relative to body CG attachment point
        glm::vec3 world = st.position + st.bodyRotation * (body->attachmentPoint() + local);
        TerrainContact tc = terrain.contactAt(world);
        if (tc.penetration > 0.f) {
            contacts.push_back({ world, tc.normal, tc.penetration });
        }
    }
    return contacts;
}

// Ground clamp: safety net to prevent vehicle from falling through terrain.
// The springs + body collision handle normal ground contact; this only catches
// catastrophic penetration (e.g., spawning inside terrain or physics explosion).
static void groundClamp(VehiclePhysics& physics, const Terrain& terrain)
{
    auto& st = physics.mutableState();
    float groundH = terrain.heightAt(st.position.x, st.position.z);

    // Body bottom is ~0.25m above vehicle origin (CG at 0.35, collider bottom at -0.10).
    // Only clamp if the body would be fully buried in terrain.
    float minY = groundH - 0.10f;
    if (st.position.y < minY) {
        st.position.y = minY;
        if (st.velocity.y < 0.f)
            st.velocity.y = 0.f;
    }
}

// Initialize the vehicle at the correct equilibrium height above terrain.
static void initVehiclePosition(VehiclePhysics& physics, const Terrain& terrain)
{
    auto& st = physics.mutableState();
    float groundH = terrain.heightAt(st.position.x, st.position.z);

    // Equilibrium: springs compressed by weight → compression = mg/(4k)
    // At rest, tire bottom = ground, tire center = ground + radius,
    // vehicle origin = tire center - compression (springs push body up, wheel stays on ground)
    float tireR = 0.31f;
    float mass  = 1300.f;
    float springRate = 35000.f;
    float eqComp = mass * 9.81f / (4.f * springRate);
    st.position.y = groundH + tireR - eqComp;
}

// Fill the Vehicle rendering struct from physics state + component tree.
static void fillVehicle(Vehicle& veh, const VehiclePhysics& physics)
{
    const auto& st = physics.state();
    veh.position     = st.position;
    veh.heading      = st.heading;
    veh.pitch        = st.pitch;
    veh.roll         = st.roll;
    veh.bodyRotation = st.bodyRotation;

    auto* phys = const_cast<VehiclePhysics*>(&physics);
    veh.frontSteerAngle = phys->frontSubframe()->left()->tire()->steerAngle();

    WheelInfo wheels[4] = {
        { phys->frontSubframe(), phys->frontSubframe()->left()  },
        { phys->frontSubframe(), phys->frontSubframe()->right() },
        { phys->rearSubframe(),  phys->rearSubframe()->left()   },
        { phys->rearSubframe(),  phys->rearSubframe()->right()  },
    };

    for (int i = 0; i < 4; ++i) {
        glm::vec3 wLocal = wheelLocalPos(wheels[i]);
        glm::vec3 mLocal = mountLocalPos(wheels[i]);

        // Account for spring compression pushing wheel up
        float comp = wheels[i].corner->spring()->compression;
        wLocal.y += comp;

        veh.wheelPos[i] = st.position + st.bodyRotation * wLocal;
        veh.mountPos[i] = st.position + st.bodyRotation * mLocal;

        const auto& to = wheels[i].corner->tire()->tireOutput();
        veh.wheelSlipRatio[i]   = to.slipRatio;
        veh.wheelSlipAngle[i]   = to.slipAngleRad;
        veh.wheelNormalLoad[i]  = to.normalLoadN;
        veh.wheelContactWidth[i] = wheels[i].corner->tire()->width;
    }
}

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

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

    // Build the playground early so we can use terrain for init height
    Playground playground = createPlayground();

    // Place vehicle at correct equilibrium height above terrain
    initVehiclePosition(physics, playground.terrain);

    AudioEngine audio;
    if (!audio.init())
        std::fprintf(stderr, "Audio init failed (continuing without sound)\n");

    Vehicle vehicle;
    TireTrails tireTrails;

    renderer.uploadTerrain(ctx, playground.terrain);

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

    // Previous spring compressions for velocity estimation
    float prevComp[4] = {};

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

            // Reset vehicle: R key or Xbox Start button
            if ((e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_r)
                || (e.type == SDL_CONTROLLERBUTTONDOWN
                    && e.cbutton.button == SDL_CONTROLLER_BUTTON_START)) {
                physics.reset();
                initVehiclePosition(physics, playground.terrain);
                tireTrails.clear();
                for (float& c : prevComp) c = 0.f;
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
        if (keys[SDL_SCANCODE_SPACE]) input.handBrake = 1.f;

        if (controller) {
            float rt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERRIGHT) / 32767.f;
            float lt = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_TRIGGERLEFT)  / 32767.f;
            float lx = SDL_GameControllerGetAxis(controller, SDL_CONTROLLER_AXIS_LEFTX)        / 32767.f;
            if (rt > 0.05f) input.throttle = rt;
            if (lt > 0.05f) input.brake    = lt;
            if (std::abs(lx) > 0.05f) input.steer = lx;
            if (SDL_GameControllerGetButton(controller, SDL_CONTROLLER_BUTTON_B))
                input.handBrake = 1.f;
        }

        // Fixed-timestep physics
        while (accumulator >= PHYSICS_DT) {
            // 1. Set spring compressions from terrain
            updateSprings(physics, playground.terrain, prevComp, PHYSICS_DT);

            // 2. Set ground normal under vehicle center
            glm::vec3 gn = playground.terrain.normalAt(
                physics.state().position.x, physics.state().position.z);
            physics.setGroundNormal(gn);

            // 3. Run physics update (forces, integration)
            physics.update(PHYSICS_DT, input);

            // 4. Body collision against terrain
            auto contacts = checkBodyCollisions(physics, playground.terrain);
            if (!contacts.empty())
                physics.applyCollisions(contacts, PHYSICS_DT);

            // 5. Ground clamp
            groundClamp(physics, playground.terrain);

            accumulator -= PHYSICS_DT;
        }

        // Fill rendering struct from physics
        fillVehicle(vehicle, physics);
        tireTrails.update(vehicle);

        // Update audio from physics state
        auto* dt_ptr = physics.drivetrain();
        audio.setEngineRpm(dt_ptr->engineRpm, dt_ptr->redlineRpm, input.throttle);
        {
            float maxSR = 0.f, maxSA = 0.f;
            for (int i = 0; i < 4; ++i) {
                maxSR = std::max(maxSR, std::abs(vehicle.wheelSlipRatio[i]));
                maxSA = std::max(maxSA, std::abs(vehicle.wheelSlipAngle[i]));
            }
            glm::vec3 bodyVel = glm::transpose(physics.state().bodyRotation) * physics.state().velocity;
            float speed = std::abs(bodyVel.z);
            float speedGate = std::clamp(speed / 3.f, 0.f, 1.f);
            audio.setTireSlip(maxSR * speedGate, maxSA * 57.2958f * speedGate);
        }

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
        glm::vec3 bodyVel = glm::transpose(physics.state().bodyRotation) * physics.state().velocity;
        hud.speedKmh = std::abs(bodyVel.z) * 3.6f;
        hud.rpm      = dt_ptr->engineRpm;
        hud.rpmLimit = dt_ptr->redlineRpm;
        hud.gear     = dt_ptr->currentGear + 1;  // 0-indexed to 1-indexed

        VkCommandBuffer cmd = ctx.beginFrame();
        if (cmd) {
            renderer.draw(cmd, ctx.swapExtent.width, ctx.swapExtent.height,
                          vehicle, hud, playground, &trailGeo);
            ctx.endFrame();
        }
    }

    vkDeviceWaitIdle(ctx.device);
    audio.shutdown();
    renderer.shutdown(ctx.device);
    ctx.shutdown();
    if (controller) SDL_GameControllerClose(controller);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
