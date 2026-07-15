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
#include "../orz_highway.h"
#include "../pibt_lock.h"
#include "RuntimeConfig.h"
#include <chrono>
#include <iostream>


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
#if ORZ_HIGHWAY_IDLE_EXIT
            if (ORZ_Highway::is_labeled(dummy_goals.at(i))) {
                // BFS to nearest unlabeled walkable cell
                std::queue<int> q; std::unordered_set<int> vis;
                q.push(dummy_goals.at(i)); vis.insert(dummy_goals.at(i));
                while (!q.empty()) {
                    int cur = q.front(); q.pop();
                    if (!ORZ_Highway::is_labeled(cur)) {
                        dummy_goals.at(i) = cur; break;
                    }
                    int col = cur % env->cols, row = cur / env->cols;
                    int nbs[4]; int nn = 0;
                    if (row > 0 && env->map[cur - env->cols] == 0) nbs[nn++] = cur - env->cols;
                    if (row < env->rows - 1 && env->map[cur + env->cols] == 0) nbs[nn++] = cur + env->cols;
                    if (col > 0 && env->map[cur - 1] == 0) nbs[nn++] = cur - 1;
                    if (col < env->cols - 1 && env->map[cur + 1] == 0) nbs[nn++] = cur + 1;
                    for (int ni = 0; ni < nn; ni++) {
                        if (vis.find(nbs[ni]) == vis.end()) {
                            vis.insert(nbs[ni]); q.push(nbs[ni]);
                        }
                    }
                }
            }
#endif
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
#if ORZ_HIGHWAY_IDLE_FOLLOW
               int _aloc = env->curr_states[i].location;
               if (ORZ_Highway::is_labeled(_aloc)) {
                   int _ex = -1;
                   uint8_t _pd;
                   if (ORZ_Highway::get_forbid(_aloc)) {
                       _ex = ORZ_Highway::get_exit(_aloc);
                   } else if (ORZ_Highway::get_guide(_aloc, _pd)) {
                       _ex = ORZ_Highway::get_guide_exit(_aloc);
                   }
                   if (_ex >= 0) dummy_goals.at(i) = _ex;
               }
#endif
               int g = dummy_goals.at(i);
               trajLNS.tasks[i] = g;
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
        // 每 tick 最多更新 50 个 agent 的 guide path
        const int MAX_GUIDE_UPDATES = 50;
        int guide_update_count = 0;

        // 第一遍：新任务优先（traj_dists 为空）
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (guide_update_count >= MAX_GUIDE_UPDATES) break;
            if (require_guide_path[i] && trajLNS.traj_dists[i].empty()){
                if (trajLNS.guide_update_on_cooldown(i)) continue;
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty()) remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &flow_end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()) add_traj(trajLNS, i);
                    if (std::chrono::steady_clock::now() >= flow_end_time) break;
                    trajLNS.mark_guide_update_failed(i, GUIDE_UPDATE_FAILURE_COOLDOWN_STEPS);
                    continue;
                }
                guide_update_count++;
            }
        }

        // 第二遍：轮询刷新已有路径的 agent（从上次停下的位置开始）
        static int rot_idx = 0;
        int start = rot_idx;
        for (int offset = 0; offset < env->num_of_agents && guide_update_count < MAX_GUIDE_UPDATES; offset++)
        {
            int i = (start + offset) % env->num_of_agents;
            if (require_guide_path[i] && !trajLNS.traj_dists[i].empty()){
                if (trajLNS.guide_update_on_cooldown(i)) continue;
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty()) remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &flow_end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()) add_traj(trajLNS, i);
                    if (std::chrono::steady_clock::now() >= flow_end_time) break;
                    trajLNS.mark_guide_update_failed(i, GUIDE_UPDATE_FAILURE_COOLDOWN_STEPS);
                    continue;
                }
                guide_update_count++;
                rot_idx = (i + 1) % env->num_of_agents;
            }
        }
        if (guide_update_count > 0 && start == rot_idx) {
            rot_idx = (rot_idx + 1) % env->num_of_agents;
        }

        // one-time FW optimization in the flow allocation phase
        auto t_fw = std::chrono::steady_clock::now();
        std::unordered_set<int> updated;
        // 关掉 Frank-Wolfe（只保留 guide A* 更新减少耗时）
        // frank_wolfe(trajLNS, updated, flow_end_time);
        long long fw_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_fw).count();
        PIBT_Lock::g_fw_ms = (int)(fw_us/1000);
        PIBT_Lock::g_guide_update_cnt = guide_update_count;
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
                { uint8_t fb = ORZ_Highway::get_forbid(env->curr_states[a].location);
                  if (fb && fb != 15) pa += 5000; }
                { uint8_t fb = ORZ_Highway::get_forbid(env->curr_states[b].location);
                  if (fb && fb != 15) pb += 5000; }
                if (ORZ_Highway::is_exit_cell(env->curr_states[a].location)) pa = 1e9;
                if (ORZ_Highway::is_exit_cell(env->curr_states[b].location)) pb = 1e9;
                if (env->goal_locations[a].empty()) pa += 0;
                if (env->goal_locations[b].empty()) pb += 0;
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

    /**
     * @brief Default planner initialization
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

        // DIAG: print guide status once
        static bool diag_printed = false;
        if (!diag_printed) {
            diag_printed = true;
            fprintf(stderr, "[DIAG] guide_enabled=%d highway_enabled=%d flow_enabled=%d target_ratio=%.2f\n",
                    (int)nz::cfg().game_guide_path_enabled,
                    (int)nz::cfg().game_highway_enabled,
                    (int)nz::cfg().game_dir_flow_enabled,
                    nz::cfg().game_dir_flow_target_ratio);
            fflush(stderr);
        }
        // Reset PIBT depth counters for this tick
        PIBT_Lock::g_pibt_max_depth = 0;
        PIBT_Lock::g_pibt_total_chains = 0;
        PIBT_Lock::g_pibt_total_depth_sum = 0;
        PIBT_Lock::g_fw_ms = 0;
        PIBT_Lock::g_guide_update_cnt = 0;

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
        auto t_setup = std::chrono::steady_clock::now();
        initialize_dummy_goals_if_needed(env);
        setup_multistep_episode_state(env, flow_end_time, local_priority);
        update_guide_paths_once_for_multistep(env, flow_end_time);
        auto t_pibt = std::chrono::steady_clock::now();
        long long guide_us = std::chrono::duration_cast<std::chrono::microseconds>(t_pibt - t_setup).count();

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

        // DIAG: PIBT depth stats every 50 calls
        static int diag_n = 0;
        if (++diag_n % 50 == 0) {
            // PIBT time = total - guide time
            long long total_us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now() - t_setup).count();
            long long pibt_us = total_us - guide_us;
            double avg = PIBT_Lock::g_pibt_total_chains > 0
                ? (double)PIBT_Lock::g_pibt_total_depth_sum / PIBT_Lock::g_pibt_total_chains
                : 0.0;
            fprintf(stderr, "[PLAN] guide=%lldms pibt=%lldms total=%lldms fw=%d upd=%d chains=%d max_depth=%d avg_depth=%.1f\n",
                    guide_us/1000, pibt_us/1000, total_us/1000,
                    PIBT_Lock::g_fw_ms, PIBT_Lock::g_guide_update_cnt,
                    PIBT_Lock::g_pibt_total_chains, PIBT_Lock::g_pibt_max_depth, avg);
            fflush(stderr);
        }
    }
}
