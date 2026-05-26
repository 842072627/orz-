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
#include <chrono>
#include <iostream>
#include <unordered_map>
#include <cmath>
#include <algorithm>


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

    // ==== 连续空间 swept-rect 碰撞检测辅助函数 ====
    static bool swept_interval(float d0, float dv, float lower, float upper,
                                float& out_t0, float& out_t1) {
        const float eps = 1e-6f;
        if (dv > -eps && dv < eps) {
            if (d0 > lower + eps && d0 < upper - eps) {
                out_t0 = 0.0f; out_t1 = 1.0f;
                return true;
            }
            return false;
        }
        float t0 = (lower - d0) / dv;
        float t1 = (upper - d0) / dv;
        if (t0 > t1) std::swap(t0, t1);
        const float teps = eps / std::fabs(dv);
        t0 += teps; t1 -= teps;
        out_t0 = std::max(0.0f, t0);
        out_t1 = std::min(1.0f, t1);
        return out_t0 <= out_t1;
    }

    // 检查两个 agent 的 FW 动作在连续空间中是否 swept-rect 碰撞
    // 输入：两个 agent 在 prev_states 和 next_states 中的位置
    // 返回 true 如果它们的 1x1 box 在 [0,1] 时间内碰撞
    static bool swept_collision_check(int i, int j,
        const std::vector<State>& prev_states,
        const std::vector<State>& next_states,
        int cols, int rows)
    {
        const float AGENT_SIZE = 1.0f;

        int pi_loc = prev_states[i].location;
        int pj_loc = prev_states[j].location;
        int ni_loc = next_states[i].location;
        int nj_loc = next_states[j].location;

        if (ni_loc < 0 || nj_loc < 0) return false;
        if (pi_loc == pj_loc) return true;  // 同格子出发 → 必然碰撞

        float xi0 = (float)(pi_loc % cols);
        float yi0 = (float)(pi_loc / cols);
        float xi1 = (float)(ni_loc % cols);
        float yi1 = (float)(ni_loc / cols);
        float xj0 = (float)(pj_loc % cols);
        float yj0 = (float)(pj_loc / cols);
        float xj1 = (float)(nj_loc % cols);
        float yj1 = (float)(nj_loc / cols);

        // 轴对齐 swept 包围盒预筛
        float ri_min_x = std::min(xi0, xi1);
        float ri_max_x = std::max(xi0, xi1) + AGENT_SIZE;
        float ri_min_y = std::min(yi0, yi1);
        float ri_max_y = std::max(yi0, yi1) + AGENT_SIZE;
        float rj_min_x = std::min(xj0, xj1);
        float rj_max_x = std::max(xj0, xj1) + AGENT_SIZE;
        float rj_min_y = std::min(yj0, yj1);
        float rj_max_y = std::max(yj0, yj1) + AGENT_SIZE;

        if (ri_min_x >= rj_max_x || ri_max_x <= rj_min_x ||
            ri_min_y >= rj_max_y || ri_max_y <= rj_min_y)
            return false;

        // 精确连续时间碰撞
        float d0x = xi0 - xj0;
        float d0y = yi0 - yj0;
        float dvx = (xi1 - xi0) - (xj1 - xj0);
        float dvy = (yi1 - yi0) - (yj1 - yj0);

        float tx0 = 0, tx1 = 0, ty0 = 0, ty1 = 0;
        bool cx = swept_interval(d0x, dvx, -AGENT_SIZE, AGENT_SIZE, tx0, tx1);
        bool cy = swept_interval(d0y, dvy, -AGENT_SIZE, AGENT_SIZE, ty0, ty1);
        return cx && cy && std::max(tx0, ty0) <= std::min(tx1, ty1);
    }

    static void run_multistep_pibt_once(SharedEnvironment* env, std::vector<double>& local_priority,
                                        std::vector<Action>& one_step_actions)
    {
        const auto start_time = std::chrono::steady_clock::now();
        std::sort(ids.begin(), ids.end(), [&](int a, int b) {
                return local_priority.at(a) > local_priority.at(b);
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

    // ==== 连续空间碰撞过滤 ====
    // 只检测对向流（同一轴线上相反方向）且距离 ≤ 2 的 FW agent 对
    {
        int cols = env->cols;

        // 收集每个 FW agent 的运动方向
        struct DirVec { int dx, dy; };
        std::vector<DirVec> fw_dir(env->num_of_agents, {0, 0});
        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int pi = prev_states[i].location;
            int ni = next_states[i].location;
            if (ni < 0) continue;
            fw_dir[i].dx = (ni % cols) - (pi % cols);
            fw_dir[i].dy = (ni / cols) - (pi / cols);
        }

        // 按 next_state 位置分 bin（只放 FW agent）
        std::unordered_map<int, std::vector<int>> bins;
        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int nloc = next_states[i].location;
            if (nloc >= 0) bins[nloc].push_back(i);
        }

        // 每个 agent 查周围 3x3 cell
        static const int dr[9] = {0, -1, 1, 0, 0, -1, -1, 1, 1};
        static const int dc[9] = {0, 0, 0, -1, 1, -1, 1, -1, 1};

        for (int i = 0; i < env->num_of_agents; i++) {
            if (one_step_actions[i] != Action::FW) continue;
            int ni = next_states[i].location;
            if (ni < 0) continue;
            int ri = ni / cols, ci = ni % cols;
            int dxi = fw_dir[i].dx, dyi = fw_dir[i].dy;

            for (int d = 0; d < 9; d++) {
                int nr = ri + dr[d], nc = ci + dc[d];
                if (nr < 0 || nr >= env->rows || nc < 0 || nc >= cols) continue;
                auto it = bins.find(nr * cols + nc);
                if (it == bins.end()) continue;

                for (int j : it->second) {
                    if (j <= i) continue;
                    if (one_step_actions[j] != Action::FW) continue;
                    if (next_states[j].location < 0) continue;

                    // === 方向性预筛：只处理对向流 ===
                    int dxj = fw_dir[j].dx, dyj = fw_dir[j].dy;

                    // 两 agent 必须在同一轴线上且方向相反
                    // 即 (dxi * dxj < 0 && dyi == 0 && dyj == 0) 水平对向
                    // 或 (dyi * dyj < 0 && dxi == 0 && dxj == 0) 垂直对向
                    bool head_on = false;
                    if (dxi != 0 && dxj != 0 && dyi == 0 && dyj == 0 && dxi * dxj < 0)
                        head_on = true;
                    else if (dyi != 0 && dyj != 0 && dxi == 0 && dxj == 0 && dyi * dyj < 0)
                        head_on = true;

                    if (!head_on) continue;

                    // === 距离预筛：在运动方向上的距离 ≤ 2 ===
                    int pi_loc = prev_states[i].location;
                    int pj_loc = prev_states[j].location;
                    int dist = 999;
                    if (dxi != 0)  // 水平对向
                        dist = std::abs(pi_loc % cols - pj_loc % cols);
                    else  // 垂直对向
                        dist = std::abs(pi_loc / cols - pj_loc / cols);
                    if (dist > 2) continue;

                    // swept-rect 检查（与 ActionModel 相同逻辑）
                    if (!swept_collision_check(i, j, prev_states, next_states, cols, env->rows))
                        continue;

                    // 碰撞！低 priority 的 wait
                    int loser, winner;
                    if (local_priority[i] < local_priority[j]) {
                        loser = i; winner = j;
                    } else {
                        loser = j; winner = i;
                    }

                    one_step_actions[loser] = Action::W;
                    int old_loc = next_states[loser].location;
                    if (old_loc >= 0 && old_loc < (int)decision.size())
                        decision[old_loc] = -1;
                    decided[loser].loc = prev_states[loser].location;
                    decided[loser].state = DONE::DONE;
                    next_states[loser] = prev_states[loser];
                }
            }
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
