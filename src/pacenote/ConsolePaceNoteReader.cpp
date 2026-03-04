#include "ConsolePaceNoteReader.h"
#include "core/Logging.h"

void ConsolePaceNoteReader::speak(const PaceNote& note)
{
    LOG_INFO("[Pacenote] %s", renderNote(note).c_str());
}
