#pragma once
#include <stdint.h>

#define FLAPPY_BIRD_W 15
#define FLAPPY_BIRD_H 10

static const uint16_t flappy_bird_rows[FLAPPY_BIRD_H] = {
    0x0038, // .....###.....
    0x00FC, // ....######...
    0x00F4, // ....###.#....
    0x07FF, // ..###########
    0x7FDE, // ########.###.
    0x3FDC, // ..########.##
    0x1E3C, // ...####...###
    0x10FC, // ...#..######.
    0x0FF8, // ....########.
    0x03F0, // .....######..
};
