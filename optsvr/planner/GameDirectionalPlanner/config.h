// Local config subset for IDEA/gamePlanner's directional game planner.
#pragma once

#include <string>
#include "MapProfile.h"
#include "SharedEnv.h"

inline bool is_ORZ(SharedEnvironment* env) {
    return env != nullptr &&
           optsvr::map_profile().type == optsvr::MapType::GAME_DOMAIN;
}

#define ORZ_PIBT_DIRECTION_BLOCK false
#define ORZ_HIGHWAY_FORCE_MOVE false
#define ORZ_HIGHWAY_IDLE_EXIT true
#define ORZ_HIGHWAY_IDLE_FOLLOW true

#define USE_WALL_HUGGING false
#define USE_WALL_HUGGING_PIBT true
#define USE_WALL_HUGGING_ASTAR true
#define WALL_HUG_WIDTH_MAX 20
#define WALL_HUG_BRANCH_BIAS_MAX 2
