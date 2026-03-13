#include "Scenario.hpp"

Playground createPlayground()
{
    Playground pg;
    pg.terrain.generate();
    return pg;
}
