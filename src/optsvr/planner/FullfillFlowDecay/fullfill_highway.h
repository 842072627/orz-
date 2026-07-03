#pragma once
// Fullfill warehouse highway per-cell direction constraints
// Mask bits: 1=W, 2=E, 4=N, 8=S
#include <cstdint>

namespace FullfillHighway {

constexpr int NUM_CELLS = 33866;

struct CellConstraint { int loc; uint8_t forbid; };

extern const CellConstraint kConstraints[NUM_CELLS];

inline uint8_t get_forbid_mask(int loc) {
    int lo = 0, hi = NUM_CELLS - 1;
    while (lo <= hi) {
        int mid = (lo + hi) / 2;
        if (kConstraints[mid].loc == loc) return kConstraints[mid].forbid;
        if (kConstraints[mid].loc < loc) lo = mid + 1; else hi = mid - 1;
    }
    return 0;
}

} // namespace FullfillHighway
