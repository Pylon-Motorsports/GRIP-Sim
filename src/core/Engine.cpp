#include "Engine.h"
#include "Logging.h"
#include "input/IInputProvider.h"
#include "road/IRoadGenerator.h"
#include "road/RoadBuilder.h"
#include "rendering/IRoadRenderer.h"
#include "rendering/Camera.h"
#include "rendering/RoadMesh.h"
#include "vehicle/IVehicleDynamics.h"
#include "vehicle/SemiRealisticVehicle.h"
#include "vehicle/MultiBodyVehicle.h"
#include "pacenote/IPaceNoteGenerator.h"
#include "pacenote/IPaceNoteReader.h"
#include <SDL2/SDL.h>
#include <glm/gtc/matrix_transform.hpp>

Engine::Engine(ModuleRegistry& registry)
    : registry_(registry)
    , camera_(std::make_unique<Camera>())
    , roadMesh_(std::make_unique<RoadMesh>())
{}

Engine::~Engine() = default;

bool Engine::initialize()
{
    // SDL init (video + events; controller init is done by SdlInputProvider)
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) < 0) {
        LOG_ERROR("SDL_Init failed: %s", SDL_GetError());
        return false;
    }

    window_ = SDL_CreateWindow("GRIP-Sim",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        1280, 720,
        SDL_WINDOW_VULKAN | SDL_WINDOW_RESIZABLE);
    if (!window_) {
        LOG_ERROR("SDL_CreateWindow failed: %s", SDL_GetError());
        return false;
    }

    // Resolve modules
    input_      = registry_.get<IInputProvider>();
    roadGen_    = registry_.get<IRoadGenerator>();
    renderer_   = registry_.get<IRoadRenderer>();
    vehicle_    = registry_.get<IVehicleDynamics>();
    noteGen_    = registry_.get<IPaceNoteGenerator>();
    noteReader_ = registry_.get<IPaceNoteReader>();

    if (!input_->initialize())      { LOG_ERROR("IInputProvider::initialize failed");      return false; }
    if (!renderer_->initialize(window_)) { LOG_ERROR("IRoadRenderer::initialize failed"); return false; }
    if (!noteReader_->initialize()) { LOG_ERROR("IPaceNoteReader::initialize failed");     return false; }

    // Build road
    buildRoad();

    // Reset car to start of stage
    vehicle_->reset({ 0.f, 0.f, 0.f }, 0.f);

    initialized_ = true;
    LOG_INFO("Engine initialized");
    return true;
}

void Engine::buildRoad()
{
    auto segments = roadGen_->generate();
    *roadMesh_ = RoadBuilder::build(segments);

    // Give centreline data to vehicle for segment tracking
    if (auto* v = dynamic_cast<SemiRealisticVehicle*>(vehicle_.get()))
        v->setCenterlinePoints(roadMesh_->centerlinePoints,
                               roadMesh_->segmentStartVertex, 12);
    else if (auto* v2 = dynamic_cast<MultiBodyVehicle*>(vehicle_.get()))
        v2->setCenterlinePoints(roadMesh_->centerlinePoints,
                                roadMesh_->segmentStartVertex, 12);

    noteGen_->loadStage(segments);
    renderer_->uploadMesh(*roadMesh_);
    roadMesh_->dirty = false;
    LOG_INFO("Road built: %zu vertices, %zu indices",
             roadMesh_->vertices.size(), roadMesh_->indices.size());
}

void Engine::run()
{
    if (!initialized_) return;

    uint64_t now      = SDL_GetPerformanceCounter();
    uint64_t freq     = SDL_GetPerformanceFrequency();
    double   lastTime = static_cast<double>(now) / static_cast<double>(freq);
    double   accumulator = 0.0;

    while (true) {
        // --- Poll input ---
        input_->poll();
        if (input_->shouldQuit()) break;

        const auto& frame = input_->frame();
        if (frame.resetCar) {
            vehicle_->reset({ 0.f, 0.f, 0.f }, 0.f);
        }

        // --- Frame timing ---
        uint64_t nowTick = SDL_GetPerformanceCounter();
        double   nowSec  = static_cast<double>(nowTick) / static_cast<double>(freq);
        double   dt      = nowSec - lastTime;
        lastTime         = nowSec;
        if (dt > MAX_FRAME_TIME) dt = MAX_FRAME_TIME;
        accumulator += dt;

        // --- Fixed-step physics ---
        while (accumulator >= PHYSICS_STEP) {
            tick(frame, static_cast<float>(PHYSICS_STEP));
            accumulator -= PHYSICS_STEP;
        }

        // --- Render ---
        const auto& state = vehicle_->state();
        int w, h;
        SDL_GetWindowSize(window_, &w, &h);
        float aspect = (h > 0) ? static_cast<float>(w) / static_cast<float>(h) : 1.f;

        camera_->update(state, static_cast<float>(dt), aspect);
        renderer_->setViewProjection(camera_->viewMatrix(), camera_->projectionMatrix());

        // Simple car transform: identity model (car is not rendered in POC, road only)
        renderer_->setCarTransform(glm::translate(glm::mat4(1.f), state.position));

        renderer_->drawFrame();
    }
}

void Engine::tick(const InputFrame& input, float dt)
{
    vehicle_->integrate(input, dt);
    const auto& state = vehicle_->state();

    // Evaluate and deliver pace notes
    auto note = noteGen_->evaluate(state);
    if (note.has_value()) {
        noteReader_->speak(*note);
    }
}

void Engine::shutdown()
{
    if (renderer_) renderer_->shutdown();
    if (input_)    input_->shutdown();
    if (noteReader_) noteReader_->shutdown();
    if (window_) { SDL_DestroyWindow(window_); window_ = nullptr; }
    SDL_Quit();
    LOG_INFO("Engine shutdown complete");
}
