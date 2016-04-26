#ifndef DMC_H
#define DMC_H

#include "conturing.h"

const int edgeTable[256][16] =
{
        {-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 8, 3, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 8, 3, -1, 1, 2, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 0, 2, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {10, 2, 3, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 8, 11, 2, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 9, 0, -1, 2, 3, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {8, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 8, 4, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 7, 9, -2, -1, -1, 1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 8, 4, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 0, 3, 7, 4, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 10, 9, -1, 8, 7, 4, -2, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 7, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {8, 4, 7, -1, 3, 11, 2, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 0, 1, -1, 8, 4, 7, -1, 2, 3, 11, -2, -1, -1, -1, -1},
        {1, 2, 4, 7, 9, 11, -2, -1, -1, -1, -1, -1-1, -1, -1, -1, -1},
        {3, 11, 10, 1, -1, 8, 7, 4, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 7, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 7, 8, -1, 0, 3, 11, 10, 9, -2, -1, -1, -1, -1, -1, -1},
        {4, 7, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 5, 4, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 5, 4, -1, 0, 8, 3, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 5, 4, 1, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 5, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 9, 5, 4, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 0, 8, -1, 1, 2, 10, -1, 4, 9, 5, -2, -1, -1, -1, -1},
        {0, 2, 4, 5, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 5, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 5, 4, -1, 2, 3, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 4, 5, -1, 0, 2, 11, 8, -2, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 0, 4, 5, 1, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 5, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 10, 1, -1, 9, 4, 5, -2, -1, -1, -1, -1, -1, -1, -1},
        {4, 9, 5, -1, 0, 1, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 5, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 4, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 7, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 5, 3, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 8, 7, 5, 9, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 0, 3, 7, 5, 9, -2, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 11, -1, 5, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 7, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 11, -1, 0, 1, 5, 7, 8, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 10, 1, -1, 8, 7, 5, 9, -2, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 7, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 7, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 6, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -1, 10, 5, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 10, 5, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {10, 5, 6, -1, 3, 8, 9, 1, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -1, 1, 2, 6, 5, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 6, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 6, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 10, 6, 5, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {10, 5, 6, -1, 0, 8, 2, 11, -2, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 0, 1, 9, -1, 10, 5, 6, -2, -1, -1, -1, -1},
        {10, 5, 6, -1, 11, 2, 8, 9, 1, -2, -1, -1, -1, -1, -1, -1},
        {1, 3, 5, 6, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 6, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 6, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 6, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {8, 7, 4, -1, 5, 6, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 6, 10, -1, 0, 3, 4, 7, -2, -1, -1, -1, -1, -1, -1, -1},
        {10, 5, 6, -1, 1, 9, 0, -1, 8, 7, 4, -2, -1, -1, -1, -1},
        {10, 5, 6, -1, 7, 4, 9, 1, 3, -2, -1, -1, -1, -1, -1, -1},
        {8, 7, 4, -1, 1, 2, 6, 5, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 7, 4, -1, 1, 2, 6, 5, -2, -1, -1, -1, -1, -1, -1},
        {8, 7, 4, -1, 0, 9, 2, 5, 6, -2, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 5, 6, 7, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 5, 6, 10, -1, 8, 7, 4, -2, -1, -1, -1, -1},
        {10, 5, 6, -1, 0, 2, 11, 7, 4, -2, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 0, 1, 9, -1, 10, 5, 6, -1, 8, 7, 4, -2},
        {10, 5, 6, -1, 7, 4, 11, 2, 1, 9, -2, -1, -1, -1, -1, -1},
        {8, 7, 4, -1, 3, 11, 6, 5, 1, -2, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 5, 6, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {8, 7, 4, -1, 6, 5, 9, 0, 11, 3, -2, -1, -1, -1, -1, -1},
        {4, 5, 6, 7, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 6, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -1, 9, 10, 6, 4, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 6, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 6, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 6, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -1, 1, 2, 4, 6, 9, -2, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 6, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 2, 3, -1, 9, 4, 10, 6, -2, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 11, 8, -1, 9, 4, 6, 10, -2, -1, -1, -1, -1, -1, -1},
        {2, 3, 11, -1, 0, 1, 4, 6, 10, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 6, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 6, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 6, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 6, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 6, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {6, 7, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 6, 7, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 6, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 6, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 6, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 6, 7, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 6, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 6, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 10, 6, 9, 7, 8, -2, -1, -1, -1, -1, -1, -1},
        {0, 2, 6, 7, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {3, 11, 2, -1, 8, 7, 0, 1, 10, 6, -2, -1, -1, -1, -1, -1},
        {1, 2, 6, 7, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 6, 7, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 6, 7, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 6, 7, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {6, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 0, 3, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 0, 9, 1, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 1, 3, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 1, 2, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 1, 2, 10, -1, 0, 3, 8, -2, -1, -1, -1, -1},
        {11, 7, 6, -1, 0, 9, 10, 2, -2, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 2, 3, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1},
        {2, 3, 6, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 6, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 3, 2, 6, 7, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 6, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 6, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 6, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 6, 7, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {6, 7, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 6, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 6, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 8, 4, 11, 6, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 6, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 8, 4, 6, 11, -2, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 0, 3, 11, 6, 4, -2, -1, -1, -1, -1, -1, -1},
        {0, 9, 10, 2, -1, 8, 4, 11, 6, -2, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 6, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 6, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 2, 3, 8, 4, 6, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 6, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 6, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 6, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 6, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 6, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {6, 7, 11, -1, 4, 5, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {6, 7, 11, -1, 4, 5, 9, -1, 0, 3, 8, -2, -1, -1, -1, -1},
        {11, 7, 6, -1, 1, 0, 5, 4, -2, -1, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 8, 3, 1, 5, 4, -2, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 4, 5, 9, -1, 1, 2, 10, -2, -1, -1, -1, -1},
        {11, 7, 6, -1, 4, 5, 9, -1, 1, 2, 10, -1, 0, 3, 8, -2},
        {11, 7, 6, -1, 0, 2, 10, 5, 4, -2, -1, -1, -1, -1, -1, -1},
        {11, 7, 6, -1, 8, 3, 2, 10, 5, 4, -2, -1, -1, -1, -1, -1},
        {4, 5, 9, -1, 3, 2, 6, 7, -2, -1, -1, -1, -1, -1, -1, -1},
        {4, 5, 9, -1, 2, 0, 8, 7, 6, -2, -1, -1, -1, -1, -1, -1},
        {3, 2, 6, 7, -1, 0, 1, 5, 4, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 5, 6, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {9, 4, 5, -1, 1, 10, 6, 7, 3, -2, -1, -1, -1, -1, -1, -1},
        {9, 4, 5, -1, 6, 10, 1, 0, 8, 7, -2, -1, -1, -1, -1, -1},
        {0, 3, 4, 5, 6, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 5, 6, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 6, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 6, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 6, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 5, 6, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 9, 5, 6, 11, 8, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -1, 9, 0, 3, 11, 6, 5, -2, -1, -1, -1, -1, -1},
        {0, 2, 5, 6, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 6, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 6, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 6, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 5, 6, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 6, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 5, 6, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 6, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 6, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 6, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 10, 11, -1, 0, 3, 8, -2, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 10, 11, -1, 0, 1, 9, -2, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 10, 11, -1, 3, 8, 9, 1, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 7, 11, -1, 0, 3, 8, -2, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 7, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 7, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 5, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 5, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 7, 3, 2, 10, 5, -2, -1, -1, -1, -1, -1, -1},
        {1, 2, 5, 7, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 5, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 5, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 5, 7, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {5, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 5, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 5, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 8, 11, 10, 4, 5, -2, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 5, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 5, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 4, 5, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 5, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 5, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 5, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 5, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -1, 8, 3, 2, 10, 5, 4, -2, -1, -1, -1, -1, -1},
        {1, 2, 4, 5, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 5, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 5, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 5, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 5, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 7, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -1, 10, 9, 4, 7, 11, -2, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 7, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 7, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 7, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 7, 9, 11, -1, 0, 3, 8, -2, -1, -1, -1, -1, -1},
        {0, 2, 4, 7, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 7, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 4, 7, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 4, 7, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 4, 7, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 4, 7, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 4, 7, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 4, 7, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 4, 7, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {4, 7, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {8, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 9, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 8, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 10, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 8, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 9, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 8, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 11, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {2, 3, 8, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 2, 9, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 2, 3, 8, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 2, 10, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {1, 3, 8, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 1, 9, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {0, 3, 8, -2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
        {-2, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1}
};

const uint vertexCount[256] =
{
    0, 1, 1, 1, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 2, 1, 2, 2, 2, 1, 2, 1, 3, 1, 2, 1, 2, 1,
    1, 2, 1, 1, 2, 3, 1, 1, 2, 2, 2, 1, 2, 2, 1, 1,
    1, 1, 1, 1, 2, 2, 1, 1, 2, 1, 2, 1, 2, 1, 1, 1,
    1, 2, 2, 2, 1, 2, 1, 1, 2, 2, 3, 2, 1, 1, 1, 1,
    2, 2, 3, 2, 2, 2, 2, 1, 3, 2, 4, 2, 2, 1, 2, 1,
    1, 2, 1, 1, 1, 2, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 2, 1, 2, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 2, 3, 2, 2, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 2, 1, 2, 2, 2, 1, 1, 1, 2, 1, 1, 1, 2, 1,
    2, 3, 2, 2, 3, 4, 2, 2, 2, 2, 2, 1, 2, 2, 1, 1,
    1, 1, 1, 1, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 2, 2, 2, 1, 2, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 2, 1, 1, 1, 1, 1,
    1, 2, 1, 1, 1, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0
};

// adjacent child cells containing the same inner face of a cell
// [plane/orientation][inner_face][child_cells]
const uint cell_face_mask[3][4][2] {
    {{3,2}, {0,1}, {4,5}, {7,6}}, // x oriented child pairs (yz plane)
    {{4,7}, {0,3}, {1,2}, {5,6}}, // y oriented child pairs (xz plane)
    {{2,6}, {1,5}, {0,4}, {3,7}}  // z oriented child pairs (xy plane)
};

// child cells containing the same inner edge of a cell
// [orientation][inner_edge][child_cells]
const uint cell_edge_mask[3][2][4] {
    {{3,0,4,7}, {2,1,5,6}}, // x oriented edges
    {{4,0,1,5}, {7,3,2,6}}, // y oriented edges
    {{2,1,0,3}, {6,5,4,7}}  // z oriented edges
};

// adjacent child cells containing the same quater face of a face
// [plane/orientation][quater_face][child_cells(fst,snd)]
const uint face_face_mask[3][4][2] {
    {{2,3}, {1,0}, {5,4}, {6,7}}, // (yz plane)
    {{7,4}, {3,0}, {2,1}, {6,5}}, // (xz plane)
    {{6,2}, {5,1}, {4,0}, {7,3}}  // (xy plane)
};

// child cells containing the same edge of a tiled face
// [plane/orientation][edge][child_cells][coarse_cell(0,1),child_cell_index]
const uint face_edge_mask[3][4][4][2] {
    {{{0,5},{0,1},{1,0},{1,4}}, {{0,6},{0,2},{1,3},{1,7}}, {{1,3},{1,0},{0,1},{0,2}}, {{1,7},{1,4},{0,5},{0,6}}}, // (yz plane)
    {{{1,0},{0,3},{0,7},{1,4}}, {{1,1},{0,2},{0,6},{1,5}}, {{1,1},{0,2},{0,3},{1,0}}, {{1,5},{0,6},{0,7},{1,4}}}, // (xz plane)
    {{{0,7},{0,4},{1,0},{1,3}}, {{0,6},{0,5},{1,1},{1,2}}, {{1,0},{0,4},{0,5},{1,1}}, {{1,3},{0,7},{0,6},{1,2}}}  // (xy plane)
};

// child cells containing the same edge of a divided edge
//[orientation][edge(fst,snd)][adjacent_child_cells(l_up, l_down, r_down, r_up)]
const uint edge_edge_mask[3][2][4] {
    {{4,7,3,0},{5,6,2,1}}, // x oriented edges
    {{1,5,4,0},{2,6,7,3}}, // y oriented edges
    {{0,3,2,1},{4,7,6,5}}  // z oriented edges
};

// edge indices defining the same edge defined by 4 cells
// [orientation][cell] -> edge index
const uint edge_of_four_cells[3][4] {
    {2,6,4,0}, // x oriented edges
    {9,10,11,8}, // y oriented edges
    {3,7,5,1}  // z oriented edges
};

// orientation of inner edges of a face
// [face_orientation][edge_orientation(0..2)]
const uint face_edge_orientation[3][4] {
    {1,1,2,2}, // (yz plane)
    {0,0,2,2}, // (xz plane)
    {0,0,1,1}  // (xy plane)
};


// edge orientations of a cell (0=x, 1=y, 2=z)
const uint edge_orientation[12] = {
    0,2,0,2,0,2,0,2,1,1,1,1
};

// shift of edge origin from cell origin
const Index edge_origin[12] = {
    Index(0,0,0), Index(1,0,0), Index(0,0,1), Index(0,0,0),
    Index(0,1,0), Index(1,1,0), Index(0,1,1), Index(0,1,0),
    Index(0,0,0), Index(1,0,0), Index(1,0,1), Index(0,0,1)
};


enum SolutionSpace {
    POINT_SPACE = 3,
    LINE_SPACE = 2,
    PLANE_SPACE = 1,
    UNDEFINED = 0 // not yet calculated
};

class QEF {
private:
    array<double,6> a;
    Vector3d b;
    double c;
public:
    SolutionSpace dimension;
    Vector3f m; // the mass center of edge intersections relative to grid origin

    QEF();

    void add(const Vector3f& normal, const Vector3f& point);
    void add(const QEF& qef);
    SolutionSpace solve(const Vector3f &m, float truncation, Vector3d& c);
    SolutionSpace solve(float truncation, Vector3d &c);
    double evaluate(const Vector3f& v) const;

};

class VertexNode {
public:
    weak_ptr<VertexNode> parent;
    Vector3f v;
    QEF qef;
    int8_t surfaceIndex;
    VertexNode() : surfaceIndex(-1) {v.setZero(3);}
    bool operator <(const VertexNode& other) const;
};

enum DMCNodeType {
    INTERNAL,
    LEAF,
    COLLAPSED
};

class DMCOctreeCell : public OctreeNode {
public:
    // stray vertices contained in this cell but not connected by edges through inner cell faces
    // those vertices are clustered in an ancestor cell
    vector<shared_ptr<VertexNode>> strayVertices;
    // the (clustered) vertices in this cell
    vector<shared_ptr<VertexNode>> vertices;
    DMCOctreeCell(uint8_t level) : OctreeNode(level)  {}
    virtual VertexNode* vertexAssignedTo(uint edgeIndex) const {return nullptr;}
    virtual bool hasChildren() const {return false;}
    virtual DMCOctreeCell* child(uint i) const {return nullptr;}
    bool isHomogeneous() const;
};

class DMCOctreeNode : public DMCOctreeCell {
public:
    array<unique_ptr<DMCOctreeCell>, 8> children;

    DMCOctreeNode(uint8_t level);
    virtual ~DMCOctreeNode();
    virtual bool hasChildren() const override;
    virtual DMCOctreeCell* child(uint i) const override;
    void removeChildren();
};

class DMCOctreeLeaf : public DMCOctreeCell {
public:
    array<int8_t, 12> edgeVertices;
    DMCOctreeLeaf(uint8_t level);
    VertexNode* vertexAssignedTo(uint edgeIndex) const override {return vertices[edgeVertices[edgeIndex]].get();}
};

class IMeshBuilder {
public:
    virtual void handle(const DMCOctreeCell* node, const Index& index) = 0;
};

class DefaultMeshBuilder : public IMeshBuilder {
protected:
    float cellGridRadius;
    Vector3f origin;
    aligned_vector3f& positions;
    vector_4_uint& offset;

public:
    DefaultMeshBuilder(float cellGridRadius, const Vector3f& origin,
                       aligned_vector3f& positions, vector_4_uint& offset) :
        cellGridRadius(cellGridRadius), origin(origin), positions(positions), offset(offset) {}
};

class DMCVerticesBuilder : public DefaultMeshBuilder {
    vector_4_uint& count;

public:
    DMCVerticesBuilder(float cellGridRadius, const Vector3f& origin,
                       aligned_vector3f& positions, vector_4_uint& offset, vector_4_uint& count)
        : DefaultMeshBuilder(cellGridRadius, origin, positions, offset), count(count) {}

    void handle(const DMCOctreeCell* node, const Index& index) override;
};

class DMCCellBuilder : public DefaultMeshBuilder {
    uint res;
public:
    DMCCellBuilder(uint res, float cellGridRadius, const Vector3f& origin,
                       aligned_vector3f& positions, vector_4_uint& offset)
        : DefaultMeshBuilder(cellGridRadius, origin, positions, offset), res(res) {}
    void handle(const DMCOctreeCell* node, const Index& index) override;
};

class DualMarchingCubes {
private:
    HermiteDataSampler* sampler;
    unique_ptr<DMCOctreeNode> root;

    bool inCell(Vector3d& pos, const Vector3d& cellOrigin, const double size) const;
    void initQEF(const int edges[], uint count, const Index& cell_index, QEF& qef) const;
    void createVertexNodes(DMCOctreeLeaf &leaf, const Index& leaf_index);
    void createOctreeNodes(DMCOctreeNode& parent, unsigned int parent_size, const Index &parent_index);


    void assignSurface(const vector<shared_ptr<VertexNode>> &vertices, int from, int to) const;
    void clusterEdge(array<DMCOctreeCell*, 4> nodes, uint orientation,
                     uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode> > &vertices);
    void clusterFace(array<DMCOctreeCell*, 2> nodes, uint orientation,
                     uint& maxSurfaceIndex, const vector<shared_ptr<VertexNode> > &vertices);
    void clusterCell(const Index &cell_index, DMCOctreeCell* node);
    void generateVertex(const Index &cell_index, uint8_t level, QEF& qef, Vector3f& v);

    void bfs(IMeshBuilder& builder, bool processEmptyNodes) const;
    void initVector(vector_4_uint& v) const;
public:
    float truncation;
    DualMarchingCubes(HermiteDataSampler& sampler) : sampler(&sampler), truncation(0.1f) {}
    void createOctree();
    void createVertexTree();
    void vertices(float voxelGridRadius, aligned_vector3f& positions,
                  vector_4_uint& offset, vector_4_uint &count) const;
    void cells(float voxelGridRadius, aligned_vector3f &positions,
                  vector_4_uint& offset) const;
};

#endif // DMC_H
