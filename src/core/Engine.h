#pragma once
#include "ModuleRegistry.h"
#include "input/InputFrame.h"
#include <memory>

// Forward declarations
class IInputProvider;
class IRoadGenerator;
class IRoadRenderer;
class IVehicleDynamics;
class IPaceNoteGenerator;
class IPaceNoteReader;
class Camera;
struct RoadMesh;

/// Main simulation loop.
/// Owns no concrete module implementations — only references through interfaces.
/// Communicates with modules via ModuleRegistry.
class Engine {
public:
    explicit Engine(ModuleRegistry& registry);
    ~Engine();

    /// Initialize all modules in dependency order.
    bool initialize();

    /// Blocking run loop: polls input, ticks physics at 120 Hz, renders at display rate.
    void run();

    /// Tear down modules in reverse order.
    void shutdown();

private:
    void tick(const InputFrame& input, float dt);
    void buildRoad();

    ModuleRegistry& registry_;

    // Cached module pointers (resolved in initialize())
    std::shared_ptr<IInputProvider>     input_;
    std::shared_ptr<IRoadGenerator>     roadGen_;
    std::shared_ptr<IRoadRenderer>      renderer_;
    std::shared_ptr<IVehicleDynamics>   vehicle_;
    std::shared_ptr<IPaceNoteGenerator> noteGen_;
    std::shared_ptr<IPaceNoteReader>    noteReader_;

    std::unique_ptr<Camera>    camera_;
    std::unique_ptr<RoadMesh>  roadMesh_;

    struct SDL_Window* window_ { nullptr };
    bool initialized_ { false };

    static constexpr double PHYSICS_STEP = 1.0 / 120.0;
    static constexpr double MAX_FRAME_TIME = 0.25;
};
