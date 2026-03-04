#pragma once
#include "IRoadGenerator.h"

/// Hard-coded 12-segment test stage for the POC.
/// Demonstrates all GRIP schema surface types used by the pace note system.
/// Swap with a procedural generator or a file/DB loader when ready.
class TestStageGenerator : public IRoadGenerator {
public:
    [[nodiscard]] std::vector<grip_schema::GripSegment> generate() override;
};
