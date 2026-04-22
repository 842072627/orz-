#pragma once
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace MyPlanner {

// A117: Simplified ReservationTable for maze narrow corridors
struct Interval {
    int t1, t2;  // [t1, t2) - safe interval
    Interval(int _t1, int _t2) : t1(_t1), t2(_t2) {}
};

class ReservationTable {
public:
    // Map from location to list of occupied intervals
    std::unordered_map<int, std::vector<Interval>> intervals;
    
    // A117: Query if a location is free at time t
    bool isFree(int location, int t) {
        auto it = intervals.find(location);
        if (it == intervals.end()) return true;
        
        for (const auto& interval : it->second) {
            if (t >= interval.t1 && t < interval.t2) {
                return false;  // occupied
            }
        }
        return true;
    }
    
    // A117: Reserve a location for time interval [t1, t2)
    void reserve(int location, int t1, int t2) {
        intervals[location].push_back(Interval(t1, t2));
    }
    
    // A117: Clear old reservations (t < current_time)
    void clearOld(int current_time) {
        for (auto& pair : intervals) {
            auto& vec = pair.second;
            vec.erase(
                std::remove_if(vec.begin(), vec.end(), 
                    [current_time](const Interval& i) { return i.t2 < current_time; }),
                vec.end()
            );
        }
    }
    
    // A117: Check if location is a narrow corridor
    static bool isNarrowCorridor(int loc, int cols, const std::vector<int>& map) {
        if (loc < 0 || loc >= (int)map.size() || map[loc] != 0) return false;
        
        int neighbors = 0;
        if (loc > 0 && map[loc-1] == 0) neighbors++;
        if (loc < (int)map.size()-1 && map[loc+1] == 0) neighbors++;
        if (loc >= cols && map[loc-cols] == 0) neighbors++;
        if (loc < (int)map.size()-cols && map[loc+cols] == 0) neighbors++;
        
        return neighbors <= 2;
    }
};

}  // namespace MyPlanner
