
#ifndef pibt_hpp
#define pibt_hpp

#include <vector>
#include <list>
#include <unordered_set>
#include <tuple>
#include <cstdint>
#include "my_Types.h"
#include "my_utils.h"
#include "my_heuristics.h"
#include "my_TrajLNS.h"
#include "my_utils.h"





namespace MyPlanner{
// Global counters for heuristic calls within PIBT (defined in pibt.cpp)
extern std::uint64_t PIBT_CNT_get_dist_2_path;
extern std::uint64_t PIBT_CNT_get_heuristic;
extern std::uint64_t PIBT_CNT_manhattan;
int get_gp_h(TrajLNS& lns, int ai, int target);


bool causalPIBT(int curr_id, int higher_id,std::vector<State>& prev_states,
	 std::vector<State>& next_states,
      std::vector<int>& prev_decision, std::vector<int>& decision, 
	  std::vector<bool>& occupied,TrajLNS& lns
	  );


Action getAction(State& prev, State& next);

Action getAction(State& prev, int next_loc, SharedEnvironment* env);

bool moveCheck(int id, std::vector<bool>& checked,
		std::vector<DCR>& decided, std::vector<Action>& actions, std::vector<int>& prev_decision);
}
#endif