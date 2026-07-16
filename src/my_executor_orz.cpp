// Default executor baseline implementation.
// (auto-generated clean copy)

#include "my_executor_orz.h"
#include "executor_heat_lock.h"

namespace MyExecutorOrz{

    std::unordered_map<int, std::list<int>> tpg;
    std::unordered_map<int, std::list<int>> temp_tpg;

    vector<int> previous_locations;
    vector<State> predicted_states;

    int window_size;
    bool first_execution = true;

    std::unordered_map<int, int> cell_chain_len;

void execute_initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    window_size = (int)env->min_planner_communication_time / (env->action_time*env->max_counter) + 1;
    previous_locations.resize(env->num_of_agents, -1);
}

vector<State> execute_process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions, SharedEnvironment* env)
{
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
        previous_locations[i] = env->system_states[i].location;
    }

    auto curr_states = predicted_states;

    int moves[4] = {1, env->cols, -1, -env->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

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
    heat_lock_update(env, staged_actions);
    // lock_dir.clear();  // 保留heat lock的方向锁

    // TPG链长dump：每100tick输出
    {
        int sys_ts = env->system_timestep;
        static int last_tpg_dump = -100;
        if (sys_ts - last_tpg_dump >= 100) {
            last_tpg_dump = sys_ts;
            char tfname[256];
            snprintf(tfname, sizeof(tfname),
                "C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\tpg_chain_%d.csv", sys_ts);
            FILE* tfp = fopen(tfname, "w");
            if (tfp) {
                fprintf(tfp, "cell,row,col,chain_len\n");
                int cols = env->cols;
                for (auto& kv : cell_chain_len) {
                    int r = kv.first / cols, c = kv.first % cols;
                    fprintf(tfp, "%d,%d,%d,%d\n", kv.first, r, c, kv.second);
                }
                fclose(tfp);
            }
        }
    }

    cell_chain_len.clear();
    vector<int> action_index(env->num_of_agents,0);
    curr_states = env->system_states;
    temp_tpg.clear();

    for (int t = 0; t < window_size; t++)
    {
        auto pre_states = curr_states;
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (staged_actions[i].size() <= action_index[i])
            {
                curr_states[i].timestep++;
                continue;
            }

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
                    action_index[i]++;
                }
                else
                {
                    new_location = curr_location;
                }
            }
            else if (curr_action == Action::CR)
            {
                new_orientation = (curr_orientation + 1) % 4;
                action_index[i]++;
            }
            else if (curr_action == Action::CCR)
            {
                new_orientation = (curr_orientation - 1) % 4;
                if (new_orientation == -1)
                    new_orientation = 3;
                action_index[i]++;
            }
            curr_states[i].location = new_location;
            curr_states[i].orientation = new_orientation;   
        }
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (t == 0)
            {
                temp_tpg[pre_states[i].location].push_back(i);
            }
            if (pre_states[i].location != curr_states[i].location)
            {
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
            staged_actions[i].resize(action_index[i]);
        }
    }
    predicted_states = curr_states;
    tpg = temp_tpg;
    return predicted_states;
}

void execute_next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command, SharedEnvironment* env)
{
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        assert(tpg[prev_location].front() == i);
        if (prev_location != curr_location && tpg[prev_location].front() == i)
        {
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

        // 先查TPG：队首直接放行（避免锁死队头）
        bool tpg_clear = tpg[next_location].empty() || tpg[next_location].front() == agent_id;

        // 方向锁检查：只拦非队首的逆行agent
        if (!tpg_clear) {
            auto lock_it = lock_dir.find(next_location);
            if (lock_it != lock_dir.end() && lock_it->second != -1) {
                int entering_dir = (curr_orientation + 2) % 4;
                if (entering_dir != lock_it->second) {
                    // 逆行 → STOP
                    agent_command[agent_id] = ExecutionCommand::STOP;
                    return false;
                }
            }
        }

        if (tpg_clear)
        {
            agent_command[agent_id] = ExecutionCommand::GO;
            return true;
        }
        else
        {
            agent_command[agent_id] = ExecutionCommand::STOP;
            // TPG链追踪
            {
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
