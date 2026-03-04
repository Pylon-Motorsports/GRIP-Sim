#pragma once
#include <optional>
#include <vector>
#include "PaceNote.h"
#include "vehicle/VehicleState.h"
#include "road/GripTypes.h"

/// Module interface for pace note generation.
/// Swap implementations for different pace note systems, loaded vs generated, etc.
class IPaceNoteGenerator {
public:
    virtual ~IPaceNoteGenerator() = default;

    /// Load the stage geometry so the generator can compute notes.
    /// Called once after IRoadGenerator::generate().
    virtual void loadStage(const std::vector<grip::GripSegment>& segments) = 0;

    /// Called every physics step. Returns a note when the car crosses a trigger distance.
    /// Each note is emitted exactly once (trigger consumed after firing).
    [[nodiscard]] virtual std::optional<PaceNote> evaluate(const VehicleState& state) = 0;
};
