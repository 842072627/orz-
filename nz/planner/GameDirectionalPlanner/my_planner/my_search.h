
#ifndef search_hpp
#define search_hpp

#include "my_Types.h"
#include "my_utils.h"
#include "my_Memory.h"
#include "my_heap.h"
#include "my_search_node.h"
#include "my_heuristics.h"

namespace MyPlanner{
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline = nullptr);
}

#endif
