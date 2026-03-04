#pragma once
#include <string>
#include <vector>
#include <optional>

/// Data structures for the .grip.json interchange format used by GRIP-Note.
/// Aligns with GRIP-Note's export schema for interoperability between apps.
/// Future: a GripNoteSqliteLoader can load authored notes directly from GRIP-Note's SQLite DB.

namespace grip_note {

struct ExportNoteEntry {
    int                      seq       { 0 };
    int                      odo       { 0 };   ///< Odometer in metres
    std::string              landmark;
    std::string              direction;          ///< "L", "R", "Keep L", "Keep R", "Straight"
    std::string              severity;           ///< "1"-"6", "Hairpin", "Square"
    std::optional<std::string> duration;         ///< "Long", "Short", or absent
    std::vector<std::string> decorators;         ///< ["Care", "Bumps", "Slippery", ...]
    std::optional<std::string> joiner;           ///< "10m", "20m", "50m", or absent
    std::optional<std::string> note;             ///< Free text
};

struct ExportNoteSet {
    int         version    { 1 };
    std::string driver;
    std::string recceDate;
};

struct ExportStage {
    std::string name;
};

struct ExportRally {
    std::string name;
    std::string date;
};

/// Root structure of a .grip.json file.
struct GripExport {
    int                          gripExportVersion { 1 };
    std::string                  exportedAt;
    ExportRally                  rally;
    ExportStage                  stage;
    ExportNoteSet                noteSet;
    std::vector<ExportNoteEntry> paceNotes;
};

} // namespace grip_note
