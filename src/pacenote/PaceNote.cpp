#include "PaceNote.h"
#include <sstream>

std::string renderNote(const PaceNote& note)
{
    std::ostringstream ss;

    // Direction
    if (note.direction == CornerDirection::Left)       ss << "Left ";
    else if (note.direction == CornerDirection::Right)  ss << "Right ";

    // Severity
    switch (note.severity) {
        case CornerSeverity::Hairpin:  ss << "Hairpin"; break;
        case CornerSeverity::One:      ss << "1"; break;
        case CornerSeverity::Two:      ss << "2"; break;
        case CornerSeverity::Three:    ss << "3"; break;
        case CornerSeverity::Four:     ss << "4"; break;
        case CornerSeverity::Five:     ss << "5"; break;
        case CornerSeverity::Six:      ss << "6"; break;
        case CornerSeverity::Straight: ss << "Straight"; break;
    }

    // Duration
    if (!note.duration.empty()) ss << " " << note.duration;

    // Decorators
    for (const auto& d : note.decorators) ss << " " << d;

    // Joiner
    if (!note.joiner.empty()) ss << " " << note.joiner;

    return ss.str();
}
