#include "core/Engine.h"
#include "core/ModuleRegistry.h"
#include "core/Logging.h"

// Interfaces
#include "road/IRoadGenerator.h"
#include "rendering/IRoadRenderer.h"
#include "vehicle/IVehicleDynamics.h"
#include "input/IInputProvider.h"
#include "pacenote/IPaceNoteGenerator.h"
#include "pacenote/IPaceNoteReader.h"

// Concrete implementations
#include "road/TestStageGenerator.h"
#include "rendering/VulkanRoadRenderer.h"
#include "vehicle/SemiRealisticVehicle.h"
#include "vehicle/VehicleParams.h"
#include "input/SdlInputProvider.h"
#include "pacenote/GripPaceNoteGenerator.h"
#include "pacenote/ConsolePaceNoteReader.h"

#ifdef _WIN32
#include "pacenote/WindowsSapiReader.h"
#endif

/// Composition root — the only place concrete module types are named.
/// Swap any line below to change the module for that system.
int main(int /*argc*/, char* /*argv*/[])
{
    ModuleRegistry registry;

    // --- Road generation ---
    registry.bind<IRoadGenerator>(
        std::make_shared<TestStageGenerator>());

    // --- Input ---
    registry.bind<IInputProvider>(
        std::make_shared<SdlInputProvider>());

    // --- Vehicle dynamics ---
    registry.bind<IVehicleDynamics>(
        std::make_shared<SemiRealisticVehicle>(VehicleParams{}));

    // --- Rendering ---
    registry.bind<IRoadRenderer>(
        std::make_shared<VulkanRoadRenderer>());

    // --- Pace notes ---
    registry.bind<IPaceNoteGenerator>(
        std::make_shared<GripPaceNoteGenerator>(50.f));  // trigger 50m ahead

    // --- Pace note reader: Windows SAPI if available, console fallback ---
#ifdef _WIN32
    auto sapiReader = std::make_shared<WindowsSapiReader>();
    registry.bind<IPaceNoteReader>(sapiReader);
#else
    registry.bind<IPaceNoteReader>(
        std::make_shared<ConsolePaceNoteReader>());
#endif

    // --- Run ---
    Engine engine(registry);
    if (!engine.initialize()) {
        LOG_ERROR("Failed to initialize engine");
        return 1;
    }
    engine.run();
    engine.shutdown();
    return 0;
}
