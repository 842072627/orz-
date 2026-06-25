// Default executor baseline implementation.
//
// The planner returns multi-step intent. This executor stages a window of actions,
// predicts future states for the next planning snapshot, and emits GO/STOP commands
// every execution tick using a Temporal Dependency Graph (TPG).
//
// Reference:
//   Ma, H., Kumar, T. S., & Koenig, S. (2017). Multi-agent path finding with
//   delay probabilities. AAAI Conference on Artificial Intelligence, Vol. 31.

#include "default_executor.h"

#include <fstream>
#include <array>
#include <cmath>

namespace DefaultPlanner{

    std::unordered_map<int, std::list<int>> tpg; //dependency graph for location visiting orders
    std::unordered_map<int, std::list<int>> temp_tpg; //temporary dependency graph for current timestep, used for process new plan

    vector<int> previous_locations; //record the previous locations of agents for tpg update
    vector<State> predicted_states;

    // the default process plan type is to keep a windoed actions to staged actions, window = min_planner_communication_time/simulator_time_limit, which means we will only execute the part of the plan that can be executed within the communication time limit.
    int window_size;
    bool first_execution = true;

    // --- Executor direction lock config ---
    static constexpr float STEP_DECAY = 0.98f;     // 步数衰减
    static constexpr float HEAT_DECAY = 0.9f;      // 时间衰减
    static constexpr float LOCK_THRESHOLD = 31.0f;   // 锁热阈值

    // --- Heat debug + direction lock data ---
    #define HEAT_DEBUG 0
    std::unordered_map<int, std::array<float, 4>> heat; // 4-dir smoothed heat per loc [E,S,W,N]
    std::unordered_map<int, std::array<float, 4>> dir_cnt; // weighted per-tick direction counts
    std::unordered_map<int, int> lock_dir; // -1=unlocked, 0-3=locked direction
    int heat_tick = 0;
    std::ofstream heat_csv;

void execute_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    window_size = (int)env->min_planner_communication_time / (env->action_time*env->max_counter) + 1; //calculate the window size based on the communication time limit and action execution time
    previous_locations.resize(env->num_of_agents, -1);
    if (HEAT_DEBUG) {
        heat_csv.open("heat_debug.csv");
        heat_csv << "tick,loc,tpg_size,de,ds,dw,dn,he,hs,hw,hn,lock\n";
    }
}

vector<State> execute_process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions, SharedEnvironment* env)
{
    // Default implementation: always append, update the predicted states based on moves
    if (predicted_states.size() != env->num_of_agents)
    {
        predicted_states = env->system_states;
    }

    if (first_execution)
    {
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (tpg.find(env->curr_states[i].location) == tpg.end())
            {
                tpg[env->curr_states[i].location].push_back(i);
            }
        }
        first_execution = false;
    }


    for (int i = 0; i < env->num_of_agents; i++)
    {
        previous_locations[i] = env->system_states[i].location; //for use of keep track of tpg in move function
    }

    auto curr_states = predicted_states;

    int moves[4] = {1, env->cols, -1, -env->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

    //first apppend all actions and tpg location to window size
    for (int timestep = 0; timestep < plan[0].size() && timestep < window_size; timestep++)
    {
        for (int i = 0; i < plan.size(); i++)
        {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;

            if (plan[i][timestep] == Action::FW)
            {
                new_location = new_location + moves[curr_states[i].orientation];
                tpg[new_location].push_back(i);
            }
            else if (plan[i][timestep] == Action::CR)
            {
                new_orientation = (curr_states[i].orientation + 1) % 4;
            }
            else if (plan[i][timestep] == Action::CCR)
            {
                new_orientation = (curr_states[i].orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;   
            curr_states[i].timestep+=1;

            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
            {
                staged_actions[i].push_back(plan[i][timestep]);
            }
        }
    }

    //now run a mcp simulation assuming no delays for a windowed size, and obtain the actions
    vector<int> action_index(env->num_of_agents,0); //keep track of the next action index for each agent in the staged actions 
    curr_states = env->system_states;
    temp_tpg.clear();

    for (int t = 0; t < window_size; t++)
    {
        auto pre_states = curr_states;
        for (int i = 0; i < env->num_of_agents; i++)
        {

            if (staged_actions[i].size() <= action_index[i])//no action left to proceed
            {
                curr_states[i].timestep++;//treat as wait
                continue;
            }

            //simulate with mcp (no delay)
            Action curr_action = staged_actions[i][action_index[i]];
            int curr_location = curr_states[i].location;
            int curr_orientation = curr_states[i].orientation;
            int new_location = curr_location;
            int new_orientation = curr_orientation;

            assert(tpg[curr_location].front() == i);

            if (curr_action == Action::FW)
            {
                new_location = curr_location + moves[curr_orientation];
                assert(!tpg[new_location].empty());
                if (tpg[new_location].front() == i)
                {
                    //you can go
                    action_index[i]++;
                }
                else
                {
                    new_location = curr_location; //cannot move because of the dependency, stay at the current location
                }
            }
            else if (curr_action == Action::CR)
            {
                new_orientation = (curr_orientation + 1) % 4;
                //you can go because it's just rotation
                action_index[i]++;
            }
            else if (curr_action == Action::CCR)
            {
                new_orientation = (curr_orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
                //you can go because it's just rotation
                action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;   
        }
        //clear dependency
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (t == 0)
            {
                temp_tpg[pre_states[i].location].push_back(i);
            }

            if (pre_states[i].location != curr_states[i].location) //agent move
            {
                //insert location into temp tpg, and pop the current location from tpg
                assert(tpg[pre_states[i].location].front() == i);
                tpg[pre_states[i].location].pop_front();
                temp_tpg[curr_states[i].location].push_back(i);
            }
        }
    }
    for (int i = 0; i < env->num_of_agents; i++)
    {
        if (action_index[i] < staged_actions[i].size())
        {
            staged_actions[i].resize(action_index[i]);//remove the actions that are not executed in the windowed simulation
        }
    }
    predicted_states = curr_states;
    tpg = temp_tpg;
    return predicted_states;
}

void execute_next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env)
{
    //update the tpg based on the current system states from last tick
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        assert(tpg[prev_location].front() == i);
        if (prev_location != curr_location && tpg[prev_location].front() == i)
        {
            //remove the agent from the previous location in tpg
            tpg[prev_location].pop_front();
            previous_locations[i] = curr_location;
        }
    }

    // --- 方向锁：全量预判 + 热度衰减 ---
    dir_cnt.clear();
    lock_dir.clear();
    int moves[4] = {1, env->cols, -1, -env->cols};
    for (int i = 0; i < env->system_states.size(); i++) {
        int sim_loc = env->system_states[i].location;
        int sim_orient = env->system_states[i].orientation;
        bool sim_valid = true;
        for (size_t step = 0; step < env->staged_actions[i].size() && sim_valid; step++) {
            float w = pow(STEP_DECAY, (float)step);
            Action a = env->staged_actions[i][step];
            if (a == Action::FW) {
                int next_loc = sim_loc + moves[sim_orient];
                if (next_loc >= 0 && next_loc < env->rows * env->cols && !env->map[next_loc]) {
                    int entering_dir = (sim_orient + 2) % 4;
                    dir_cnt[next_loc][entering_dir] += w;
                    sim_loc = next_loc;
                } else {
                    sim_valid = false;
                }
            } else if (a == Action::CR) {
                sim_orient = (sim_orient + 1) % 4;
            } else if (a == Action::CCR) {
                sim_orient = (sim_orient - 1 + 4) % 4;
            }
        }
    }
    // 热度更新 + 锁决策
    for (auto& kv : dir_cnt) {
        int loc = kv.first;
        float de = kv.second[0], ds = kv.second[1], dw = kv.second[2], dn = kv.second[3];
        auto& h = heat[loc];
        float he = h[0] = h[0] * HEAT_DECAY + de;
        float hs = h[1] = h[1] * HEAT_DECAY + ds;
        float hw = h[2] = h[2] * HEAT_DECAY + dw;
        float hn = h[3] = h[3] * HEAT_DECAY + dn;
        float max_h = he; int max_d = 0;
        if (hs > max_h) { max_h = hs; max_d = 1; }
        if (hw > max_h) { max_h = hw; max_d = 2; }
        if (hn > max_h) { max_h = hn; max_d = 3; }
        int lk = (max_h >= LOCK_THRESHOLD) ? max_d : -1;
        lock_dir[loc] = lk;
        if (HEAT_DEBUG) {
            int tpg_sz = tpg.count(loc) ? (int)tpg[loc].size() : 0;
            if (tpg_sz > 0 || de + ds + dw + dn > 0.001f || lk != -1) {
                heat_csv << heat_tick << "," << loc << ","
                         << tpg_sz << ","
                         << de << "," << ds << "," << dw << "," << dn << ","
                         << he << "," << hs << "," << hw << "," << hn << ","
                         << lk << "\n";
            }
        }
    }
    heat_tick++;

    for (int i = 0; i < env->system_states.size(); i++)
    {
        mcp(i, agent_command, env);
    }
    
}

bool mcp(int agent_id, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env)
{
    if (env->staged_actions[agent_id].empty())
    {
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }
    else if (env->staged_actions[agent_id].front() != Action::FW) 
    {
        agent_command[agent_id] = ExecutionCommand::GO;
        return false;
    }
    else
    {
        int moves[4] = {1, env->cols, -1, -env->cols};
        int curr_location = env->system_states[agent_id].location;
        int curr_orientation = env->system_states[agent_id].orientation;
        int next_location = curr_location + moves[curr_orientation];

        // === 方向锁检查（当前格热区 + 全路径预判） ===
        {
            int sim_loc = curr_location;
            int sim_orient = curr_orientation;
            bool sim_valid = true;
            for (size_t step = 0; step < env->staged_actions[agent_id].size() && sim_valid; step++) {
                Action a = env->staged_actions[agent_id][step];
                if (a == Action::FW) {
                    // a) 离开当前格前，检当前格热区
                    auto cur_h = heat.find(sim_loc);
                    if (cur_h != heat.end()) {
                        float max_h = cur_h->second[0]; int max_d = 0;
                        for (int d = 1; d < 4; d++) {
                            if (cur_h->second[d] > max_h) { max_h = cur_h->second[d]; max_d = d; }
                        }
                        if (max_h >= LOCK_THRESHOLD) {
                            int flow_dir = (max_d + 2) % 4;
                            if (sim_orient != flow_dir) {
                                bool has_backup = (tpg[sim_loc].size() > 1);
                                if (!has_backup) {
                                    agent_command[agent_id] = ExecutionCommand::STOP;
                                    return false;
                                }
                            }
                        }
                    }
                    // b) 进入下一个格前，检目标格锁
                    int fut_loc = sim_loc + moves[sim_orient];
                    if (fut_loc >= 0 && fut_loc < env->rows * env->cols && !env->map[fut_loc]) {
                        auto it = lock_dir.find(fut_loc);
                        if (it != lock_dir.end() && it->second != -1) {
                            int lock_d = it->second;
                            int opp_dir = (lock_d + 2) % 4;
                            int entering_dir = (sim_orient + 2) % 4;
                            if (entering_dir == opp_dir) {
                                agent_command[agent_id] = ExecutionCommand::STOP;
                                return false;
                            }
                        }
                        sim_loc = fut_loc;
                    } else {
                        sim_valid = false;
                    }
                } else if (a == Action::CR) {
                    sim_orient = (sim_orient + 1) % 4;
                } else if (a == Action::CCR) {
                    sim_orient = (sim_orient - 1 + 4) % 4;
                }
            }
        }

        if (tpg[next_location].empty() || tpg[next_location].front() == agent_id)
        {
            agent_command[agent_id] = ExecutionCommand::GO;
            return true;
        }
        else
        {
            agent_command[agent_id] = ExecutionCommand::STOP;
            return false;
        }
    }
    
}


}