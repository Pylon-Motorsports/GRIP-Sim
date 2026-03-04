#pragma once
#include "IPaceNoteGenerator.h"
#include <vector>

/// Generates pace notes from GRIP segment geometry.
/// Emits one note per corner segment when the car is within triggerDistanceM ahead.
/// Severity and direction are derived from HorizontalAngleDeltaDegrees.
/// Decorators are added when roadside features are present.
class GripPaceNoteGenerator : public IPaceNoteGenerator {
public:
    explicit GripPaceNoteGenerator(float triggerDistanceM = 50.f);

    void loadStage(const std::vector<grip_schema::GripSegment>& segments) override;
    [[nodiscard]] std::optional<PaceNote> evaluate(const VehicleState& state) override;

private:
    float triggerDistanceM_;

    struct NoteEntry {
        PaceNote note;
        float    triggerOdoM;   ///< Odometer value at which to emit this note
        bool     fired { false };
    };

    std::vector<NoteEntry> entries_;

    static CornerSeverity  severityFromAngle(float angleDeg);
    static CornerDirection directionFromAngle(float angleDeg);
    static std::string     durationFromAngle(float angleDeg, float lengthM);
    static std::vector<std::string> decoratorsFromSegment(const grip_schema::GripSegment& seg);
};
