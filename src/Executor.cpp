#include "Executor.h"

void Executor::initialize(int preprocess_time_limit)
{
    tpg.resize(env->map.size());
    previous_locations.resize(env->num_of_agents, -1);
    if (process_plan_type == APPEND_WINDOW)
    {
        window_size = env->min_planner_communication_time / (env->action_time*env->max_counter); //calculate the window size based on the communication time limit and action execution time
        if (window_size < 1)
            window_size = 1;
    }
    else if (process_plan_type == APPEND_ONE)
    {
        window_size = 1;
    }
    else
    {
        window_size = INT_MAX;
    }
    // cout<<"executor initialized with window size: "<<window_size<<endl;
}

vector<State> Executor::process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>> & staged_actions)
{
    // Default implementation: always append, update the predicted states based on moves
    if (predicted_states.size() != env->num_of_agents)
    {
        predicted_states = env->system_states;
    }

    auto curr_states = predicted_states;

    if (env->system_timestep == 0)
    {
        //insert start locations to tpg
        for (int i = 0; i < env->num_of_agents; i++)
        {
            tpg[env->curr_states[i].location].push_back(i);
            previous_locations[i] = env->system_states[i].location;
        }
    }

    int moves[4] = {1, env->cols, -1, -env->cols};
    std::vector<std::vector<Action>> plan = plan_struct.actions;

    // //check how many actions we need to insert to ensure agents have enough actions to execute within the communication time limit.
    // int current_left_action_size = INT_MAX;
    // for (int i = 0; i < env->num_of_agents; i++)
    // {
    //     current_left_action_size = min(current_left_action_size, (int)staged_actions[i].size());
    // }
    // int num_insert_timesteps = window_size - current_left_action_size;

    // cout<<"current_left_action_size: "<<current_left_action_size<<endl;
    // cout<<"num_insert_timesteps: "<<num_insert_timesteps<<endl;

    //apppend actions to window size
    for (int timestep = 0; timestep < plan[0].size(); timestep++)
    {
        for (int i = 0; i < plan.size(); i++)
        {
            int new_location = curr_states[i].location;
            int new_orientation = curr_states[i].orientation;
            if (plan[i][timestep] == Action::FW)
            {
                new_location = new_location + moves[curr_states[i].orientation];
                tpg[new_location].push_back(i);
                // cout<<"insert "<<i<<" to tpg at "<<new_location<<endl;
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
            // cout<<"agent "<<i<<" current prediceted state location from last iteration "<<predicted_states[i].location<<" orientation "<<predicted_states[i].orientation<<endl;
            // cout<<"agent "<<i<<" action "<<(plan[i][timestep] == Action::FW ? "FW" : (plan[i][timestep] == Action::CR ? "CR" : (plan[i][timestep] == Action::CCR ? "CCR" : (plan[i][timestep] == Action::NA ? "NA" : "W"))))<<" new prediceted state location "<<new_location<<" orientation "<<new_orientation<<endl;
            predicted_states[i].location = new_location;
            predicted_states[i].orientation = new_orientation;
            predicted_states[i].timestep+=1;
            curr_states[i] = predicted_states[i];
            // cout<<"agent "<<i<<" predicted state location "<<predicted_states[i].location<<" orientation "<<predicted_states[i].orientation<<" timestep "<<predicted_states[i].timestep<<endl;
            if (plan[i][timestep] != Action::NA && plan[i][timestep] != Action::W)
            {
                staged_actions[i].push_back(plan[i][timestep]);
            }
            //cout<<" t "<<timestep<<" agent "<<i<<" current location "<<curr_states[i].location<<" predict location "<<predicted_states[i].location<<endl;
        }
    }
    return predicted_states;
}

void Executor::next_command(int exec_time_limit, std::vector<ExecutionCommand> & agent_command)
{
    // cout<<"executor next_command with exec_time_limit: "<<exec_time_limit<<endl;
    // // //always go if there are staged actions
    // for (int i = 0; i < env->curr_states.size(); i++)
    // {
    //     if (!env->staged_actions[i].empty())
    //     {
    //         agent_command[i] = ExecutionCommand::GO;
    //     }
    //     else
    //     {
    //         agent_command[i] = ExecutionCommand::STOP;
    //     }
    // }
    // return;

    //update the tpg based on the current system states from last tick
    for (int i = 0; i < env->system_states.size(); i++)
    {
        int prev_location = previous_locations[i];
        int curr_location = env->system_states[i].location;
        // cout<<"tpg for agent "<<i<<" prev loc "<< prev_location<<" tpg first "<<tpg[prev_location].front()<<endl;
        assert(tpg[prev_location].front() == i);
        if (prev_location != curr_location && tpg[prev_location].front() == i)
        {
            //remove the agent from the previous location in tpg
            // cout<<"pop agent "<<i<<" from location "<< prev_location<<" current at "<<curr_location<<endl;
            tpg[prev_location].pop_front();
            previous_locations[i] = curr_location;
        }
    }
    vector<bool> decided(env->num_of_agents, false);
    //temp_tpg = tpg; //copy the tpg to temp_tpg for current timestep processing, we will update temp_tpg during mcp but keep tpg unchanged
    for (int i = 0; i < env->system_states.size(); i++)
    {
        // always try to go and clear orders because we don't know the current delay
        if (!decided[i])
            mcp(i, decided, agent_command);
    }

}

bool Executor::mcp(int agent_id, vector<bool> & curr_decision, std::vector<ExecutionCommand> & agent_command)
{
    // ARCH-B R6: R306 Reversed Priority MCP combined with R3 space-filling Scheduler
    // The R3 Scheduler prefers space-filling (low congestion destination).
    // This Executor prioritizes FURTHER-from-goal agents to clear bottlenecks.
    // Together: space-filling assignment + reversed priority execution = better throughput.

    if (env->staged_actions[agent_id].empty())
    {
        agent_command[agent_id] = ExecutionCommand::STOP;
        curr_decision[agent_id] = true;
        return false;
    }
    else if (env->staged_actions[agent_id].front() != Action::FW)
    {
        agent_command[agent_id] = ExecutionCommand::GO;
        curr_decision[agent_id] = true;
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
            agent_command[agent_id] = ExecutionCommand::GO;
            curr_decision[agent_id] = true;
            return true;
        }
        else
        {
            int blocking_agent_id = tpg[next_location].front();

            // J57: Adaptive priority based on map size
            // Large maps (>1000 agents): Use S1 FIFO to prevent congestion collapse
            // Small maps (<=1000 agents): Use J11 priority (goal*10 + task_dist)
            bool is_large_map = (env->num_of_agents > 1000);
            
            int agent_goal_dist = 0;
            int blocking_goal_dist = 0;
            int agent_task_dist = 0;
            int blocking_task_dist = 0;

            if (agent_id < (int)env->goal_locations.size() && !env->goal_locations[agent_id].empty()) {
                int goal = env->goal_locations[agent_id].front().first;
                agent_goal_dist = std::abs(next_location - goal);
                agent_task_dist = std::abs(env->system_states[agent_id].location - goal);
            }
            if (blocking_agent_id < (int)env->goal_locations.size() && !env->goal_locations[blocking_agent_id].empty()) {
                int goal = env->goal_locations[blocking_agent_id].front().first;
                int blocking_loc = env->system_states[blocking_agent_id].location;
                blocking_goal_dist = std::abs(blocking_loc - goal);
                blocking_task_dist = std::abs(blocking_loc - goal);
            }

            if (is_large_map) {
                // S1 FIFO: no priority for large maps (prevents congestion collapse)
                agent_command[agent_id] = ExecutionCommand::STOP;
                curr_decision[agent_id] = true;
                return false;
            } else {
                // J11 Priority for small maps: prefer agents closer to completing tasks
                int agent_priority = agent_goal_dist * 10 + agent_task_dist;
                int blocking_priority = blocking_goal_dist * 10 + blocking_task_dist;
                
                if (agent_priority > blocking_priority) {
                    agent_command[agent_id] = ExecutionCommand::GO;
                    curr_decision[agent_id] = true;
                    return true;
                } else {
                    agent_command[agent_id] = ExecutionCommand::STOP;
                    curr_decision[agent_id] = true;
                    return false;
                }
            }
        }
    }
}
