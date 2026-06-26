// Default executor baseline implementation.
//
// The planner returns multi-step intent. This executor stages a window of actions,
// predicts future states for the next planning snapshot, and emits GO/STOP commands
// every execution tick using a Temporal Dependency Graph (TPG).
//
// Reference:
//   Ma, H., Kumar, T. S., & Koenig, S. (2017). Multi-agent path finding with
//   delay probabilities. AAAI Conference on Artificial Intelligence, Vol. 31.

#include "my_executor_orz.h"
#include "executor_heat_lock.h"

namespace MyExecutorOrz{

    std::unordered_map<int, std::list<int>> tpg; //dependency graph for location visiting orders
    std::unordered_map<int, std::list<int>> temp_tpg; //temporary dependency graph for current timestep, used for process new plan

    vector<int> previous_locations; //record the previous locations of agents for tpg update
    vector<State> predicted_states;

    // the default process plan type is to keep a windoed actions to staged actions, window = min_planner_communication_time/simulator_time_limit, which means we will only execute the part of the plan that can be executed within the communication time limit.
    int window_size;
    bool first_execution = true;

    // TPG链长追踪
    std::unordered_map<int, int> cell_chain_len;

void execute_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    window_size = (int)env->min_planner_communication_time / (env->action_time*env->max_counter) + 1; //calculate the window size based on the communication time limit and action execution time
    previous_locations.resize(env->num_of_agents, -1);
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
    cell_chain_len.clear();
    heat_lock_update(env, staged_actions);
    lock_dir.clear();
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
                // 方向感知锁：链≥3的拥塞格只拦逆流
                bool lock_ok = true;
                if (new_location >= 0 && new_location < env->rows*env->cols && !env->map[new_location]) {
                    auto cit = cell_chain_len.find(new_location);
                    if (cit != cell_chain_len.end() && cit->second >= 3) {
                        auto dit = dir_cnt.find(new_location);
                        if (dit != dir_cnt.end()) {
                            int flow_dir = 0;
                            float max_f = dit->second[0];
                            for (int d = 1; d < 4; d++) {
                                if (dit->second[d] > max_f) { max_f = dit->second[d]; flow_dir = d; }
                            }
                            if (curr_orientation == DIR_OPP[flow_dir]) {
                                lock_ok = false; // 逆流截停
                            }
                        }
                    }
                }
                if (lock_ok && tpg[new_location].front() == i)
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
                // cout<<\"inserting \"<<i<<\" into tpg loc \"<<pre_states[i].location<<endl;
            }

            if (pre_states[i].location != curr_states[i].location) //agent move
            {
                //insert location into temp tpg, and pop the current location from tpg
                assert(tpg[pre_states[i].location].front() == i);
                tpg[pre_states[i].location].pop_front();
                temp_tpg[curr_states[i].location].push_back(i);
                // cout<<\"inserting \"<<i<<\" into tpg loc \"<<curr_states[i].location<<endl;
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

    for (int i = 0; i < env->system_states.size(); i++)
    {
        mcp(i, agent_command, env);
    }
    
}

bool mcp(int agent_id, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env)
{
    if (env->staged_actions[agent_id].empty())
    {
        //no action, just stop and wait for the next plan, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }
    else if (env->staged_actions[agent_id].front() != Action::FW) 
    {
        //try to go but still stay at current location because it\'s not move forward, no tpg order clear
        agent_command[agent_id] = ExecutionCommand::GO;
        return false;
    }
    else
    {
        int moves[4] = {1, env->cols, -1, -env->cols};
        int curr_location = env->system_states[agent_id].location;
        int curr_orientation = env->system_states[agent_id].orientation;
        int next_location = curr_location + moves[curr_orientation];

        if (tpg[next_location].empty() || tpg[next_location].front() == agent_id)
        {
            //try to go and can go, clear the tpg order
            agent_command[agent_id] = ExecutionCommand::GO;
            return true;
        }
        else
        {
                //the blocking agent cannot go, so we cannot go
                agent_command[agent_id] = ExecutionCommand::STOP;
                // TPG链追踪
                {
                    int sys_ts = env->system_timestep;
                    int chain = 1;
                    int blocker = tpg[next_location].front();
                    int cur_blk = blocker;
                    int visited[20000] = {0};
                    int visited_count = 0;
                    visited[visited_count++] = agent_id;
                    visited[visited_count++] = cur_blk;
                    while (chain < 20 && cur_blk >= 0 && cur_blk < (int)env->system_states.size()) {
                        int blk_loc = env->system_states[cur_blk].location;
                        int blk_ori = env->system_states[cur_blk].orientation;
                        int blk_next = blk_loc + moves[blk_ori];
                        if (blk_next >= 0 && blk_next < env->rows * env->cols && !env->map[blk_next]) {
                            auto it = tpg.find(blk_next);
                            if (it != tpg.end() && !it->second.empty() && it->second.front() != cur_blk) {
                                cur_blk = it->second.front();
                                bool seen = false;
                                for (int v = 0; v < visited_count; v++) {
                                    if (visited[v] == cur_blk) { seen = true; break; }
                                }
                                if (seen) break;
                                visited[visited_count++] = cur_blk;
                                chain++;
                                continue;
                            }
                        }
                        break;
                    }
                    if (chain > cell_chain_len[next_location])
                        cell_chain_len[next_location] = chain;
                }
                return false;
        }
    }
    
}

}
