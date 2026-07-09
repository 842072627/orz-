#pragma once
// 93 guide cells — 100000 penalty to opposite EW direction
// guide direction = preferred travel direction
// ew_dir: 0=E (penalize east for guide=l), 2=W (penalize west for guide=r)
#include <cstdint>
#include <algorithm>

namespace orz_engine {

constexpr int IRON_HIGHWAY_GUIDE_COUNT = 93;
struct GuideEntry { int loc; uint8_t ew_dir; };

constexpr GuideEntry kHighwayGuide[93] = {
    {1783615, 2},
    {1783616, 2},
    {1783617, 2},
    {1783618, 2},
    {1783619, 2},
    {1783620, 2},
    {1783621, 2},
    {1783622, 2},
    {1783623, 2},
    {1783624, 2},
    {1785415, 2},
    {1787215, 2},
    {1789014, 0},
    {1789015, 0},
    {1790815, 0},
    {1835812, 0},
    {1859229, 2},
    {1859230, 2},
    {1859231, 2},
    {1859232, 2},
    {1859233, 2},
    {1859234, 2},
    {1859235, 2},
    {1861035, 2},
    {1862835, 2},
    {1864635, 2},
    {1866435, 2},
    {1868235, 2},
    {1870035, 2},
    {1871835, 0},
    {1871836, 0},
    {1873635, 0},
    {1873636, 0},
    {1875435, 0},
    {1875436, 0},
    {1877235, 0},
    {1877236, 0},
    {1879035, 0},
    {1879036, 0},
    {1880835, 0},
    {1880836, 0},
    {1882635, 0},
    {1882636, 0},
    {1884435, 0},
    {1884436, 0},
    {1886235, 0},
    {1886236, 0},
    {1888022, 0},
    {1888023, 0},
    {1888024, 0},
    {1888025, 0},
    {1888026, 0},
    {1888027, 0},
    {1888028, 0},
    {1888029, 0},
    {1888030, 0},
    {1888031, 0},
    {1888032, 0},
    {1888033, 0},
    {1888034, 0},
    {1888035, 0},
    {1889822, 2},
    {1889823, 2},
    {1889824, 2},
    {1889825, 2},
    {1889826, 2},
    {1889827, 2},
    {1889828, 2},
    {1889829, 2},
    {1889830, 2},
    {1889831, 2},
    {1889832, 2},
    {1889833, 2},
    {1889834, 2},
    {1889835, 2},
    {1889836, 2},
    {1891636, 2},
    {1893436, 2},
    {1895236, 2},
    {1897036, 2},
    {1898836, 2},
    {1900636, 2},
    {1902429, 2},
    {1902430, 2},
    {1902431, 2},
    {1902432, 2},
    {1902433, 2},
    {1902434, 2},
    {1902435, 2},
    {1902436, 2},
    {1909607, 2},
    {1911407, 2},
    {1911408, 2},
};

inline bool get_highway_guide(int loc, uint8_t& ew_dir) {
    struct Cmp { bool operator()(const GuideEntry& a, int b) { return a.loc < b; }
                 bool operator()(int a, const GuideEntry& b) { return a < b.loc; } };
    auto it = std::lower_bound(kHighwayGuide, kHighwayGuide + IRON_HIGHWAY_GUIDE_COUNT, loc, Cmp());
    if (it != kHighwayGuide + IRON_HIGHWAY_GUIDE_COUNT && it->loc == loc) {
        ew_dir = it->ew_dir;
        return true;
    }
    return false;
}

} // namespace orz_engine
