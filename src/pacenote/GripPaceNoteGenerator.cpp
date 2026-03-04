#include "GripPaceNoteGenerator.h"
#include <cmath>
#include <numeric>

GripPaceNoteGenerator::GripPaceNoteGenerator(float triggerDistanceM)
    : triggerDistanceM_(triggerDistanceM)
{}

// ---------------------------------------------------------------------------
// Severity mapping — aligned with GRIP-Note's severity values
// ---------------------------------------------------------------------------
CornerSeverity GripPaceNoteGenerator::severityFromAngle(float angleDeg)
{
    float abs = std::abs(angleDeg);
    if (abs >= 120.f) return CornerSeverity::Hairpin;
    if (abs >=  90.f) return CornerSeverity::One;
    if (abs >=  60.f) return CornerSeverity::Two;
    if (abs >=  45.f) return CornerSeverity::Three;
    if (abs >=  30.f) return CornerSeverity::Four;
    if (abs >=  15.f) return CornerSeverity::Five;
    if (abs >    0.5f) return CornerSeverity::Six;
    return CornerSeverity::Straight;
}

CornerDirection GripPaceNoteGenerator::directionFromAngle(float angleDeg)
{
    if (angleDeg < -0.5f) return CornerDirection::Left;
    if (angleDeg >  0.5f) return CornerDirection::Right;
    return CornerDirection::None;
}

std::string GripPaceNoteGenerator::durationFromAngle(float angleDeg, float lengthM)
{
    float abs = std::abs(angleDeg);
    if (abs < 0.5f) {
        // Straight — classify by length
        if (lengthM >= 80.f) return "Long";
        if (lengthM >= 40.f) return "";
    }
    // Corner — classify by arc length relative to angle
    if (lengthM / std::max(1.f, abs) > 1.5f) return "Long";
    return "";
}

std::vector<std::string> GripPaceNoteGenerator::decoratorsFromSegment(const grip_schema::GripSegment& seg)
{
    std::vector<std::string> decs;
    // Add "Care" if there are features close to the road
    auto hasFeature = [&](const std::vector<grip_schema::RoadsideFeature>& feats) {
        for (const auto& f : feats)
            if (f.distanceFromRoadCenterCentimeters < 150) return true;
        return false;
    };
    if (hasFeature(seg.leftFeatures) || hasFeature(seg.rightFeatures))
        decs.push_back("Care");
    return decs;
}

// ---------------------------------------------------------------------------
// loadStage — compute note entries from segment geometry
// ---------------------------------------------------------------------------
void GripPaceNoteGenerator::loadStage(const std::vector<grip_schema::GripSegment>& segments)
{
    entries_.clear();

    float odo = 0.f;
    int   seq = 0;

    for (size_t i = 0; i < segments.size(); ++i) {
        const auto& seg = segments[i];
        CornerSeverity  sev = severityFromAngle(seg.horizontalAngleDeltaDegrees);
        CornerDirection dir = directionFromAngle(seg.horizontalAngleDeltaDegrees);

        PaceNote note;
        note.seq          = seq++;
        note.odoMeters    = static_cast<int>(odo);
        note.severity     = sev;
        note.direction    = dir;
        note.duration     = durationFromAngle(seg.horizontalAngleDeltaDegrees,
                                              seg.centerLineLengthMeters);
        note.decorators   = decoratorsFromSegment(seg);
        note.segmentIndex = static_cast<int>(i);
        note.distanceMeters = triggerDistanceM_;

        // Tightens/opens: compare with previous corner
        if (i > 0 && dir != CornerDirection::None) {
            const auto& prevSeg = segments[i - 1];
            CornerSeverity prevSev = severityFromAngle(prevSeg.horizontalAngleDeltaDegrees);
            CornerDirection prevDir = directionFromAngle(prevSeg.horizontalAngleDeltaDegrees);
            if (prevDir == dir) {
                if (static_cast<int>(sev) < static_cast<int>(prevSev))
                    note.decorators.insert(note.decorators.begin(), "Tightens");
                else if (static_cast<int>(sev) > static_cast<int>(prevSev))
                    note.decorators.insert(note.decorators.begin(), "Opens");
            }
        }

        // Joiner: short segment after a corner
        if (i + 1 < segments.size()) {
            float nextLen = segments[i + 1].centerLineLengthMeters;
            if (seg.centerLineLengthMeters > 0.f &&
                seg.horizontalAngleDeltaDegrees != 0.f &&
                nextLen < 30.f)
            {
                note.joiner = std::to_string(static_cast<int>(nextLen)) + "m";
            }
        }

        NoteEntry entry;
        entry.note          = note;
        entry.triggerOdoM   = odo - triggerDistanceM_;  // fire this many metres before the corner
        entry.fired         = false;
        entries_.push_back(entry);

        odo += seg.centerLineLengthMeters;
    }
}

// ---------------------------------------------------------------------------
// evaluate — called every physics step
// ---------------------------------------------------------------------------
std::optional<PaceNote> GripPaceNoteGenerator::evaluate(const VehicleState& state)
{
    for (auto& entry : entries_) {
        if (!entry.fired && state.odoMeters >= entry.triggerOdoM) {
            entry.fired = true;
            if (entry.note.severity != CornerSeverity::Straight)
                return entry.note;
            // For straights: only emit if it's a long straight (duration = "Long")
            if (!entry.note.duration.empty())
                return entry.note;
        }
    }
    return std::nullopt;
}
