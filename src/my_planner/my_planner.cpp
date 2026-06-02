// Default planner baseline implementation.
//
// For each planning episode, this module updates guide paths with a bounded
// traffic-flow phase (Frank-Wolfe optimisation), then runs MY_PIBT to produce
// a multi-step plan. Internal rollout builds future steps; the environment
// snapshot is restored after planning.
//
// References:
//   Chen, Z., Harabor, D., Li, J., & Stuckey, P. J. (2024). Traffic flow
//   optimisation for lifelong multi-agent path finding. AAAI Conference on
//   Artificial Intelligence, Vol. 38, No. 18, pp. 20674-20682.
//   https://ojs.aaai.org/index.php/AAAI/article/view/30054/31856
//
//   Okumura, K., et al. (2022). Priority Inheritance with Backtracking (MY_PIBT)
//   for iterative multi-agent path finding. Artificial Intelligence, Vol. 310.

#include "my_planner.h"
#include "my_heuristics.h"
#include "SharedEnv.h"
#include "my_pibt.h"
#include "my_flow.h"
#include "my_const.h"
#include "../pibt_lock.h"
#include "../config.h"
#include "../refuge_table.h"
#include <chrono>
#include <iostream>
#include <cmath>


namespace MyPlanner{

    static const char* debug_action_to_string(Action action)
    {
        switch (action)
        {
            case Action::FW: return "FW";
            case Action::CR: return "CR";
            case Action::CCR: return "CCR";
            case Action::W: return "W";
            case Action::NA: return "NA";
            default: return "UNKNOWN";
        }
    }

    //default planner data
    std::vector<int> decision; 
    std::vector<int> prev_decision;
    std::vector<double> p;
    std::vector<State> prev_states;
    std::vector<State> next_states;
    std::vector<int> ids;
    std::vector<double> p_copy;
    std::vector<bool> occupied;
    std::vector<DCR> decided;
    std::vector<bool> checked;
    std::vector<bool> require_guide_path;
    std::vector<int> dummy_goals;
    TrajLNS trajLNS;
    std::mt19937 mt1;

    static State rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
    {
        State next = state;
        next.timestep = state.timestep + 1;

        if (action == Action::CR)
        {
            next.orientation = (state.orientation + 1) % 4;
            return next;
        }
        if (action == Action::CCR)
        {
            next.orientation = (state.orientation + 3) % 4;
            return next;
        }
        if (action != Action::FW)
        {
            return next;
        }

        int delta = 0;
        if (state.orientation == 0) delta = 1;
        if (state.orientation == 1) delta = env->cols;
        if (state.orientation == 2) delta = -1;
        if (state.orientation == 3) delta = -env->cols;

        int next_loc = state.location + delta;
        if (next_loc >= 0 && next_loc < env->map.size() && validateMove(state.location, next_loc, env))
        {
            next.location = next_loc;
        }
        return next;
    }

    static void initialize_dummy_goals_if_needed(SharedEnvironment* env)
    {
        if (env->curr_timestep != 0)
            return;
        dummy_goals.resize(env->num_of_agents);
        for(int i=0; i<env->num_of_agents; i++)
        {
            dummy_goals.at(i) = env->start_states.at(i).location;
        }
    }

    static void setup_multistep_episode_state(SharedEnvironment* env, TimePoint flow_end_time,
                                              std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        for(int i=0; i<env->num_of_agents; i++)
        {
            // initialise heuristic tables for goals
            if (std::chrono::steady_clock::now() < flow_end_time){
                for (int j = 0; j < env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.find(goal_loc) == trajLNS.heuristics.end() ||
                        trajLNS.heuristics[goal_loc].empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
                    }
                }
            }

            // set goals/tasks
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
            }

            // detect guide-path mismatch (goal changed or no guide path)
            require_guide_path[i] = false;
            if (trajLNS.trajs[i].empty() || trajLNS.trajs[i].back() != trajLNS.tasks[i])
                require_guide_path[i] = true;

            // update per-agent transient planning state
            assert(env->start_states[i].location >=0);
            prev_states[i] = env->start_states[i];
            next_states[i] = State();
            prev_decision[env->start_states[i].location] = i;
            if (decided[i].loc == -1){
                decided[i].loc = env->start_states[i].location;
                assert(decided[i].state == DONE::DONE);
            }
            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            // cross-episode priority update
            if(require_guide_path[i])
                local_priority[i] = p_copy[i];
            else if (!env->goal_locations[i].empty())
                local_priority[i] = local_priority[i]+1;

            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->start_states[i].location].size() == 1){
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }

    static void update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
    {
        // one-time guide-path update for goal-changed agents
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (std::chrono::steady_clock::now() > flow_end_time)
                break;
            if (require_guide_path[i]){
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &flow_end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()){
                        add_traj(trajLNS, i);
                    }
                    break;
                }
            }
        }

        // one-time FW optimization in the flow allocation phase
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated, flow_end_time);
    }

    static void refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
    {
        prev_decision.clear();
        prev_decision.resize(env->map.size(), -1);
        for(int i=0; i<env->num_of_agents; i++)
        {
            if (env->goal_locations[i].empty()){
                trajLNS.tasks[i] = dummy_goals.at(i);
                local_priority[i] = p_copy[i];
            }
            else{
                trajLNS.tasks[i] = env->goal_locations[i].front().first;
                local_priority[i] = local_priority[i] + 1;
            }

            assert(env->start_states[i].location >=0);
            prev_states[i] = env->start_states[i];
            next_states[i] = State();
            prev_decision[env->start_states[i].location] = i;

            if (prev_states[i].location == decided[i].loc){
                decided[i].state = DONE::DONE;
            }
            if (decided[i].state == DONE::NOT_DONE){
                decision.at(decided[i].loc) = i;
                next_states[i] = State(decided[i].loc,-1,-1);
            }

            if (!env->goal_locations[i].empty() && trajLNS.neighbors[env->start_states[i].location].size() == 1){
                local_priority[i] = local_priority[i] + 10;
            }
        }
    }

    static void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                        std::vector<Action>& one_step_actions)
    {
        const auto start_time = std::chrono::steady_clock::now();
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                double pa = local_priority.at(a);
                double pb = local_priority.at(b);
#if USE_TIDAL_REFUGE
                // Tidal priority: against-flow agents get -delta, like champion planner
                if (a < PIBT_Lock::MAX_AGENTS && PIBT_Lock::g_agent_refuge[a]) pa -= 1000.0;
                if (b < PIBT_Lock::MAX_AGENTS && PIBT_Lock::g_agent_refuge[b]) pb -= 1000.0;
#endif
                return pa > pb;
            }
        );
        //clear reset occupied
        std::fill(occupied.begin(), occupied.end(), false);

        for (int i : ids){
            if (decided[i].state == DONE::NOT_DONE){
                continue;
            }
            if (next_states[i].location==-1){
                assert(prev_states[i].location >=0 && prev_states[i].location < env->map.size());
                causalMY_PIBT(i,-1,prev_states,next_states,
                    prev_decision,decision,
                    occupied, trajLNS);
            }
        }

        one_step_actions.resize(env->num_of_agents);
        for (int id : ids){
            if (next_states.at(id).location!= -1)
                decision.at(next_states.at(id).location) = -1;
            if (next_states.at(id).location >=0){
                decided.at(id) = DCR({next_states.at(id).location,DONE::NOT_DONE});
            }
            one_step_actions.at(id) = getAction(prev_states.at(id),decided.at(id).loc, env);
            checked.at(id) = false;
        }

        for (int id=0;id < env->num_of_agents ; id++){
            if (!checked.at(id) && one_step_actions.at(id) == Action::FW){
                moveCheck(id,checked,decided,one_step_actions,prev_decision);
            }
        }

        prev_states = next_states;

        const auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now() - start_time).count();
    }

    static void append_actions_and_rollout_states(SharedEnvironment* env,
                                                   std::vector<std::vector<Action>> & actions,
                                                   const std::vector<Action>& one_step_actions)
    {
        for (int aid = 0; aid < env->num_of_agents; aid++){
            actions[aid].push_back(one_step_actions[aid]);
        }

        std::vector<State> rolled_states(env->num_of_agents);
        for (int aid = 0; aid < env->num_of_agents; aid++){
            rolled_states[aid] = rollout_next_state(env->start_states[aid], one_step_actions[aid], env);
        }
        env->start_states = rolled_states;
        env->curr_timestep += 1;
    }

    void tidal_refuge_check(SharedEnvironment* env, const TimePoint* deadline) {
#if USE_TIDAL_REFUGE
        // ——— Per-zone refuge heat tracking ———
        static std::vector<int> s_zone_prev_density;
        static std::vector<char> s_zone_refuge_active;
        int n_zones = CenterlineAccess::g_n24r * CenterlineAccess::g_n24c;
        if ((int)s_zone_prev_density.size() != n_zones) {
            s_zone_prev_density.assign(n_zones, 0);
            s_zone_refuge_active.assign(n_zones, 0);
        }
        // Update zone refuge status from density
        if (CenterlineAccess::g_zone24_density && (int)CenterlineAccess::g_zone24_density->size() >= n_zones) {
            for (int zi = 0; zi < n_zones; zi++) {
                int d = (*CenterlineAccess::g_zone24_density)[zi];
                if (d >= ZONE_REFUGE_HEAT_ON && d > s_zone_prev_density[zi])
                    s_zone_refuge_active[zi] = 1;
                else if (d < ZONE_REFUGE_HEAT_OFF)
                    s_zone_refuge_active[zi] = 0;
                s_zone_prev_density[zi] = d;
            }
        }

        // Log active zones every 100 steps
        if (env->curr_timestep % 100 == 0) {
            for (int zi = 0; zi < n_zones; zi++) {
                if (s_zone_refuge_active[zi])
                    std::cout << "[REFUGE] step=" << env->curr_timestep
                              << " zone=" << zi
                              << " density=" << (*CenterlineAccess::g_zone24_density)[zi]
                              << " prev=" << s_zone_prev_density[zi]
                              << std::endl;
            }
        }

        if (!CenterlineAccess::g_mp_width_p10 || !CenterlineAccess::g_mp_lock_dir ||
            !CenterlineAccess::g_main_dist || !CenterlineAccess::g_mp_main_dist ||
            CenterlineAccess::g_mp_count <= 0 || CenterlineAccess::g_n24c <= 0) return;
        
        for (int aid = 0; aid < env->num_of_agents; aid++) {
            if (aid >= PIBT_Lock::MAX_AGENTS) break;
            if (PIBT_Lock::g_agent_refuge[aid]) continue;
            if (trajLNS.trajs[aid].size() < 2) continue;
            int agent_loc = env->start_states[aid].location;
            if (agent_loc < 0) continue;
            int a_m = (*CenterlineAccess::g_main_dist)[agent_loc];
            if (a_m < 0) continue;
            
            for (int si = 0; si < CenterlineAccess::g_mp_count; si++) {
                int w = (*CenterlineAccess::g_mp_width_p10)[si];
                if (w <= 0 || w > TIDAL_WIDTH_TH) continue;
                
                int seg_end = si;
                while (seg_end+1 < CenterlineAccess::g_mp_count && 
                       (*CenterlineAccess::g_mp_width_p10)[seg_end+1] <= TIDAL_WIDTH_TH) seg_end++;
                int lo_m = (*CenterlineAccess::g_mp_main_dist)[si];
                int hi_m = (*CenterlineAccess::g_mp_main_dist)[seg_end];
                if (lo_m > hi_m) { int t=lo_m; lo_m=hi_m; hi_m=t; }
                
                int lock_dir = (*CenterlineAccess::g_mp_lock_dir)[si];
                if (lock_dir == 0) { si = seg_end; continue; }
                if (a_m >= lo_m && a_m <= hi_m) { si = seg_end; continue; }
                
                int dir = (a_m < lo_m) ? 1 : -1;
                if (dir == lock_dir) { si = seg_end; continue; }
                
                // Guide path crosses bottleneck?
                bool crosses = false;
                for (auto loc : trajLNS.trajs[aid]) {
                    if (loc < 0) continue;
                    int m = (*CenterlineAccess::g_main_dist)[loc];
                    if (m >= lo_m && m <= hi_m) { crosses = true; break; }
                }
                if (!crosses) { si = seg_end; continue; }
                
                // ——— Zone heat gate: only apply refuge if the bottleneck's zone is hot and rising ———
                int mp_loc = (*CenterlineAccess::g_mp_loc)[si];
                if (mp_loc >= 0) {
                    int mp_row = mp_loc / env->cols;
                    int mp_col = mp_loc % env->cols;
                    int zone_id = (mp_row / 24) * CenterlineAccess::g_n24c + (mp_col / 24);
                    if (zone_id >= 0 && zone_id < n_zones && !s_zone_refuge_active[zone_id]) {
                        si = seg_end;  // this bottleneck's zone isn't congested — skip
                        continue;
                    }
                }
                
                // Find refuge entry closest to agent along main axis, in lock direction
                int best_ti = -1; int best_d = 999999;
                for (int ti = 0; ti < REFUGE_TABLE_SIZE; ti++) {
                    int entry_mid = (REFUGE_TABLE[ti].lo_m + REFUGE_TABLE[ti].hi_m) / 2;
                    int entry_dir = (a_m < entry_mid) ? 1 : -1;
                    if (entry_dir != lock_dir) continue;
                    if (a_m >= REFUGE_TABLE[ti].lo_m && a_m <= REFUGE_TABLE[ti].hi_m) continue;
                    int boundary = (lock_dir > 0) ? REFUGE_TABLE[ti].lo_m : REFUGE_TABLE[ti].hi_m;
                    int d = (lock_dir > 0) ? (boundary - a_m) : (a_m - boundary);
                    if (d < 20) continue;
                    if (d < best_d) { best_d = d; best_ti = ti; }
                }
                if (best_ti < 0) { si = seg_end; continue; }
                
                // Get refuge coordinates
                int rx = (lock_dir > 0) ? REFUGE_TABLE[best_ti].lo_x : REFUGE_TABLE[best_ti].hi_x;
                int ry = (lock_dir > 0) ? REFUGE_TABLE[best_ti].lo_y : REFUGE_TABLE[best_ti].hi_y;
                if (rx < 0 || ry < 0) { si = seg_end; continue; }
                
                int target_loc = (ry + 1) * env->cols + (rx + 1);
                if (target_loc < 0 || target_loc >= (int)env->map.size() || env->map[target_loc] != 0) {
                    target_loc = ry * env->cols + rx;
                    if (target_loc < 0 || target_loc >= (int)env->map.size() || env->map[target_loc] != 0)
                        { si = seg_end; continue; }
                }
                
                // === REROUTE GUIDE PATH TO REFUGE (champion planner pattern) ===
                // Save old guide path
                auto base_traj = trajLNS.trajs[aid];
                auto base_gn = trajLNS.goal_nodes[aid];
                int base_task = trajLNS.tasks[aid];
                
                // Remove old path, set refuge as task, compute new path
                remove_traj(trajLNS, aid);
                trajLNS.tasks[aid] = target_loc;
                if (update_traj(trajLNS, aid, deadline)) {
                    // Refuge path computed successfully — set flag for priority + forbids_move
                    PIBT_Lock::g_agent_refuge[aid] = 1;
                } else {
                    // Restore original guide path
                    trajLNS.tasks[aid] = base_task;
                    trajLNS.trajs[aid] = std::move(base_traj);
                    trajLNS.goal_nodes[aid] = base_gn;
                    add_traj(trajLNS, aid);
                    update_dist_2_path(trajLNS, aid);
                }
                break;
            }
        }
#endif
    }

    /**
     * @brief Default planner initialize function
     * 
     * @param preprocess_time_limit time limit for preprocessing in milliseconds
     * @param env shared environment object
     * 
     * The initialization function initializes the default planner data structures and heuristics tables.
     */
    void initialize(int preprocess_time_limit, SharedEnvironment* env){
            //initialise all required data structures
            assert(env->num_of_agents != 0);
            p.resize(env->num_of_agents);
            decision.resize(env->map.size(), -1);
            prev_states.resize(env->num_of_agents);
            next_states.resize(env->num_of_agents);
            decided.resize(env->num_of_agents,DCR({-1,DONE::DONE}));
            occupied.resize(env->map.size(),false);
            checked.resize(env->num_of_agents,false);
            ids.resize(env->num_of_agents);
            require_guide_path.resize(env->num_of_agents,false);
            for (int i = 0; i < ids.size();i++){
                ids[i] = i;
            }

            // initialise the heuristics tables containers
            init_heuristics(env);
            mt1.seed(0);
            srand(0);

            new (&trajLNS) TrajLNS(env, global_heuristictable, global_neighbors);
            trajLNS.init_mem();

            //assign intial priority to each agent
            std::shuffle(ids.begin(), ids.end(), mt1);
            for (int i = 0; i < ids.size();i++){
                p[ids[i]] = ((double)(ids.size() - i))/((double)(ids.size()+1));
            }
            p_copy = p;
            return;
    };

    /**
     * @brief Default planner plan function
     * 
     * @param time_limit time limit for planning in milliseconds
     * @param actions vector of actions to be populated by the planner
     * @param env shared environment object
     * @param num_steps number of steps to plan
     * 
     * The plan function is the main function of the default planner. 
     * It computes the actions for the agents based on the current state of the environment.
     * The function first checks assignments/goal location changes and perform the necessary updates.
     * It then computes and optimises traffic flow optimised guide paths for the agents.
     * Finally, it computes the actions for the agents using MY_PIBT that follows the guide path heuristics and returns the actions.
     * Note that the default planner ignores the turning action costs, and post-processes turning actions as additional delays on top of original plan.
     */
    void plan(int time_limit, std::vector<std::vector<Action>> & actions,
                         SharedEnvironment* env, int num_steps){
        actions.clear();
        //cout<<"num of steps to plan "<<num_steps<<endl;
        if (env == nullptr || env->num_of_agents <= 0){
            return;
        }

        actions.resize(env->num_of_agents);
        if (num_steps <= 0){
            return;
        }

        const auto episode_start = std::chrono::steady_clock::now();
        const TimePoint episode_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit));
        int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents/100;
        if (pibt_time <= 0){
            pibt_time = 1;
        }
        const int flow_budget_ms = std::max(0, time_limit - std::max(MIN_PIBT_TIME, pibt_time * num_steps) - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
        TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

        std::vector<double> local_priority = p;

        const std::vector<State> original_states = env->start_states;
        const int original_timestep = env->curr_timestep;
        const std::vector<DCR> original_decided = decided;

        // Fresh execution tracking for each multi-step planning episode:
        // no ongoing orientation-compensation actions are carried across episodes.
        decided.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

        // --- One-time setup for this planning episode ---
        initialize_dummy_goals_if_needed(env);
        setup_multistep_episode_state(env, flow_end_time, local_priority);
        update_guide_paths_once_for_multistep(env, flow_end_time);
        tidal_refuge_check(env, &flow_end_time);

        const auto after_setup = std::chrono::steady_clock::now();
        const auto setup_elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(after_setup - episode_start).count();
        // commit only cross-episode priority update (internal rollout keeps using local_priority only)
        p = local_priority;

        for (int step = 0; step < num_steps; step++){
            if (std::chrono::steady_clock::now() >= episode_deadline){
                break;
            }
            std::vector<Action> one_step_actions;
            // After step 0, only run MY_PIBT + rollout (no guide path recompute and no FW)
            if (step > 0){
                refresh_multistep_step_state(env, local_priority);
            }
            run_multistep_pibt_once(env, local_priority, one_step_actions);
            append_actions_and_rollout_states(env, actions, one_step_actions);
        }

        // restore env state after internal rollout simulation
        env->start_states = original_states;
        env->curr_timestep = original_timestep;
        decided = original_decided;
    }
}
