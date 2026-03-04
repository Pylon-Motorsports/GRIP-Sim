#pragma once
#include <string>
#include <vector>

/// Severity aligned with GRIP-Note's severity field values.
enum class CornerSeverity : uint8_t {
    Hairpin = 1, ///< >= 120 deg
    One     = 2, ///< >= 90 deg
    Two     = 3, ///< >= 60 deg
    Three   = 4, ///< >= 45 deg
    Four    = 5, ///< >= 30 deg
    Five    = 6, ///< >= 15 deg
    Six     = 7, ///< > 0 deg (fast sweep)
    Straight= 8  ///< no significant angle
};

/// Direction aligned with GRIP-Note's direction field values.
enum class CornerDirection : uint8_t { Left, Right, None };

/// Pace note data aligned with GRIP-Note's pace_notes SQL table.
/// Fields match the .grip.json export format for interoperability.
struct PaceNote {
    // -- GRIP-Note schema fields --
    int                      seq           { 0 };
    int                      odoMeters     { 0 };   ///< Odometer at trigger point (metres)
    std::string              landmark;               ///< Optional landmark name
    CornerDirection          direction     { CornerDirection::None };
    CornerSeverity           severity      { CornerSeverity::Straight };
    std::string              duration;               ///< "Long", "Short", or empty
    std::vector<std::string> decorators;             ///< e.g. {"Care", "Bumps"}
    std::string              joiner;                 ///< e.g. "10m", "20m", or empty
    std::string              note;                   ///< Free-text annotation

    // -- Sim-internal fields --
    float distanceMeters { 0.f };  ///< How far ahead this note triggers (lookahead)
    int   segmentIndex   { -1 };
};

/// Format a PaceNote as a co-driver call string.
/// Matches GRIP-Note's renderNote() format: [direction] [severity] [duration] [decorators] [joiner]
std::string renderNote(const PaceNote& note);
