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
#include "RuntimeConfig.h"
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <iostream>
#include <mutex>
#include <thread>
#include <unordered_map>


namespace MyPlanner{

    namespace {

    int resolve_dist2path_prewarm_threads(int configured_threads)
    {
        if (configured_threads > 0) return configured_threads;
        const unsigned int hc = std::thread::hardware_concurrency();
        if (hc <= 1) return 1;
        return static_cast<int>(hc - 1);
    }

    void prewarm_dist_2_path_radius(Dist2Path& dp, const SharedEnvironment* env,
                                    int radius,
                                    const std::atomic<bool>& cancel_requested)
    {
        if (radius <= 0 || env == nullptr) return;
        std::vector<int> predecessors;
        predecessors.reserve(4);
        while (!dp.open.empty()) {
            if (cancel_requested.load(std::memory_order_relaxed)) return;
            const d2p curr = dp.open.front();
            if (curr.cost >= radius) return;
            dp.open.pop_front();

            predecessors.clear();
            const int candidates[4] = {
                curr.id - env->cols,
                curr.id + env->cols,
                curr.id - 1,
                curr.id + 1
            };
            for (int prev : candidates) {
                if (prev < 0 || prev >= static_cast<int>(env->map.size())) continue;
                if (orz_directional_move_allowed(prev, curr.id, env)) {
                    predecessors.push_back(prev);
                }
            }

            for (int prev : predecessors) {
                const int cost = curr.cost + 1;
                auto it = dp.dist2path.find(prev);
                if (it != dp.dist2path.end() &&
                    it->second.label == dp.label && cost >= it->second.cost) {
                    continue;
                }
                dp.open.emplace_back(dp.label, prev, cost, curr.togo);
                dp.dist2path.insert_or_assign(
                    prev, d2p(dp.label, prev, cost, curr.togo));
            }
        }
    }

    class Dist2PathPrewarmer {
    public:
        Dist2PathPrewarmer(SharedEnvironment* env, int radius, int configured_threads)
            : env_(env),
              radius_(std::max(0, radius)),
              thread_count_(resolve_dist2path_prewarm_threads(configured_threads)),
              slots_(env != nullptr ? env->num_of_agents : 0)
        {
            if (env_ == nullptr || radius_ <= 0 || slots_.empty()) {
                enabled_ = false;
                thread_count_ = 0;
                return;
            }
            enabled_ = true;
            workers_.reserve(thread_count_);
            for (int i = 0; i < thread_count_; ++i) {
                workers_.emplace_back([this]() { worker_loop(); });
            }
        }

        ~Dist2PathPrewarmer()
        {
            request_stop(true);
            join_workers();
        }

        Dist2PathPrewarmer(const Dist2PathPrewarmer&) = delete;
        Dist2PathPrewarmer& operator=(const Dist2PathPrewarmer&) = delete;

        int radius() const { return radius_; }
        int thread_count() const { return thread_count_; }

        void enqueue(int agent, const Traj& path)
        {
            if (!enabled_ || path.empty() ||
                agent < 0 || agent >= static_cast<int>(slots_.size())) {
                return;
            }
            std::lock_guard<std::mutex> lock(mu_);
            if (!accepting_) return;
            Slot& slot = slots_[agent];
            slot.latest_path_version++;
            slot.path = path;
            auto old_result = results_.find(agent);
            if (old_result != results_.end() &&
                old_result->second.path_version != slot.latest_path_version) {
                results_.erase(old_result);
                discarded_++;
            }
            enqueued_++;

            if (slot.running) {
                slot.dirty = true;
                return;
            }
            if (!slot.queued) {
                slot.queued = true;
                queue_.push_back(agent);
                cv_.notify_one();
            }
        }

        bool finish_until(TimePoint deadline)
        {
            if (!enabled_) return true;
            bool completed = false;
            {
                std::unique_lock<std::mutex> lock(mu_);
                accepting_ = false;
                completed = done_cv_.wait_until(lock, deadline, [this]() {
                    return queue_.empty() && active_jobs_ == 0;
                });
                if (!completed) {
                    cancel_requested_.store(true, std::memory_order_relaxed);
                    queue_.clear();
                    for (Slot& slot : slots_) {
                        slot.queued = false;
                        slot.dirty = false;
                    }
                }
                stopping_ = true;
            }
            cv_.notify_all();
            join_workers();
            return completed;
        }

        int commit(TrajLNS& lns)
        {
            if (!enabled_) return 0;
            int committed = 0;
            for (auto& item : results_) {
                const int agent = item.first;
                if (agent < 0 || agent >= static_cast<int>(lns.traj_dists.size()) ||
                    agent >= static_cast<int>(slots_.size())) {
                    continue;
                }
                const Slot& slot = slots_[agent];
                if (item.second.path_version != slot.latest_path_version ||
                    slot.built_path_version != slot.latest_path_version) {
                    discarded_++;
                    continue;
                }
                lns.traj_dists[agent] = std::move(item.second.dp);
                committed++;
            }
            results_.clear();
            return committed;
        }

        int enqueued() const { return enqueued_; }
        int built() const { return built_; }
        int discarded() const { return discarded_; }
        bool cancelled() const
        {
            return cancel_requested_.load(std::memory_order_relaxed);
        }

    private:
        struct Slot {
            int latest_path_version = 0;
            int built_path_version = 0;
            bool queued = false;
            bool running = false;
            bool dirty = false;
            Traj path;
        };

        struct Result {
            int path_version = 0;
            Dist2Path dp;
        };

        void request_stop(bool cancel)
        {
            if (!enabled_) return;
            {
                std::lock_guard<std::mutex> lock(mu_);
                accepting_ = false;
                stopping_ = true;
                if (cancel) {
                    cancel_requested_.store(true, std::memory_order_relaxed);
                }
            }
            cv_.notify_all();
            done_cv_.notify_all();
        }

        void join_workers()
        {
            for (std::thread& worker : workers_) {
                if (worker.joinable()) worker.join();
            }
            workers_.clear();
        }

        void worker_loop()
        {
            while (true) {
                int agent = -1;
                int path_version = 0;
                Traj path;
                {
                    std::unique_lock<std::mutex> lock(mu_);
                    cv_.wait(lock, [this]() {
                        return stopping_ || !queue_.empty();
                    });
                    if (stopping_ && queue_.empty()) return;
                    agent = queue_.front();
                    queue_.pop_front();
                    Slot& slot = slots_[agent];
                    slot.queued = false;
                    slot.running = true;
                    slot.dirty = false;
                    path_version = slot.latest_path_version;
                    path = slot.path;
                    active_jobs_++;
                }

                Result result;
                result.path_version = path_version;
                if (!path.empty() &&
                    !cancel_requested_.load(std::memory_order_relaxed)) {
                    init_dist_2_path(result.dp, env_, path);
                    prewarm_dist_2_path_radius(
                        result.dp, env_, radius_, cancel_requested_);
                }

                {
                    std::lock_guard<std::mutex> lock(mu_);
                    active_jobs_--;
                    Slot& slot = slots_[agent];
                    slot.running = false;
                    if (!cancel_requested_.load(std::memory_order_relaxed) &&
                        !slot.dirty && slot.latest_path_version == path_version) {
                        slot.built_path_version = path_version;
                        results_[agent] = std::move(result);
                        built_++;
                    } else {
                        discarded_++;
                        if (!stopping_ && slot.latest_path_version != path_version &&
                            !slot.queued) {
                            slot.queued = true;
                            queue_.push_back(agent);
                            cv_.notify_one();
                        } else if (!stopping_ && slot.dirty && !slot.queued) {
                            slot.queued = true;
                            queue_.push_back(agent);
                            cv_.notify_one();
                        }
                    }
                    if (queue_.empty() && active_jobs_ == 0) {
                        done_cv_.notify_all();
                    }
                }
            }
        }

        SharedEnvironment* env_ = nullptr;
        int radius_ = 0;
        int thread_count_ = 0;
        bool enabled_ = false;

        std::mutex mu_;
        std::condition_variable cv_;
        std::condition_variable done_cv_;
        bool accepting_ = true;
        bool stopping_ = false;
        int active_jobs_ = 0;
        std::atomic<bool> cancel_requested_{false};

        std::deque<int> queue_;
        std::vector<Slot> slots_;
        std::vector<std::thread> workers_;
        std::unordered_map<int, Result> results_;

        int enqueued_ = 0;
        int built_ = 0;
        int discarded_ = 0;
    };

    } // namespace

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
            if (optsvr::cfg().game_highway_enabled && ORZ_Highway::is_labeled(dummy_goals.at(i))) {
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
            if (optsvr::cfg().game_guide_path_enabled &&
                std::chrono::steady_clock::now() < flow_end_time){
                for (int j = 0; j < env->goal_locations[i].size(); j++)
                {
                    int goal_loc = env->goal_locations[i][j].first;
                    if (trajLNS.heuristics.find(goal_loc) == trajLNS.heuristics.end() ||
                        trajLNS.heuristics[goal_loc].empty()){
                        init_heuristic(trajLNS.heuristics[goal_loc], env, goal_loc);
                    }
                }
            }
            const int old_task = trajLNS.tasks[i];
            // set goals/tasks
            if (env->goal_locations[i].empty()){
#if ORZ_HIGHWAY_IDLE_FOLLOW
               int _aloc = env->curr_states[i].location;
               if (optsvr::cfg().game_highway_enabled && ORZ_Highway::is_labeled(_aloc)) {
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
            if (optsvr::cfg().game_guide_path_enabled &&
                (trajLNS.trajs[i].empty() ||
                 trajLNS.trajs[i].back() != trajLNS.tasks[i]))
                require_guide_path[i] = true;
            const bool goal_changed = old_task != trajLNS.tasks[i];

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
            if (require_guide_path[i] ||
                (!optsvr::cfg().game_guide_path_enabled && goal_changed))
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
        if (!optsvr::cfg().game_guide_path_enabled) return;

        const int prewarm_radius = optsvr::cfg().dist2path_prewarm_enabled
            ? optsvr::cfg().dist2path_prewarm_radius
            : 0;
        Dist2PathPrewarmer prewarmer(
            env, prewarm_radius, optsvr::cfg().dist2path_prewarm_threads);
        int prewarm_skipped_by_time = 0;
        auto enqueue_prewarm = [&](int agent) {
            const auto left_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                flow_end_time - std::chrono::steady_clock::now()).count();
            if (left_ms < std::max(
                    0, optsvr::cfg().dist2path_prewarm_min_left_ms)) {
                prewarm_skipped_by_time++;
                return;
            }
            prewarmer.enqueue(agent, trajLNS.trajs[agent]);
        };

        // one-time guide-path update for goal-changed agents
        for (int i = 0; i < env->num_of_agents; i++)
        {
            if (std::chrono::steady_clock::now() > flow_end_time)
                break;
            if (require_guide_path[i]){
                if (trajLNS.guide_update_on_cooldown(i)){
                    continue;
                }
                std::vector<int> old_traj = trajLNS.trajs[i];
                if (!trajLNS.trajs[i].empty())
                    remove_traj(trajLNS, i);
                if (!update_traj(trajLNS, i, &flow_end_time)){
                    trajLNS.trajs[i] = old_traj;
                    if (!old_traj.empty()){
                        add_traj(trajLNS, i);
                    }
                    if (std::chrono::steady_clock::now() >= flow_end_time){
                        break;
                    }
                    trajLNS.mark_guide_update_failed(i, GUIDE_UPDATE_FAILURE_COOLDOWN_STEPS);
                    continue;
                }
                enqueue_prewarm(i);
            }
        }

        // one-time FW optimization in the flow allocation phase
        std::unordered_set<int> updated;
        frank_wolfe(trajLNS, updated, flow_end_time);
        for (int agent : updated) {
            enqueue_prewarm(agent);
        }

        const auto wait_start = std::chrono::steady_clock::now();
        const bool prewarm_completed = prewarmer.finish_until(flow_end_time);
        const int prewarm_committed = prewarmer.commit(trajLNS);
        const int wait_ms = static_cast<int>(
            std::chrono::duration_cast<std::chrono::milliseconds>(
                std::chrono::steady_clock::now() - wait_start).count());

        if (optsvr::cfg().timing_trace) {
            std::cerr << "[TIMING][GAME_PREWARM]"
                      << " q=" << prewarmer.enqueued()
                      << " built=" << prewarmer.built()
                      << " commit=" << prewarm_committed
                      << " discard=" << prewarmer.discarded()
                      << " skip=" << prewarm_skipped_by_time
                      << " wait=" << wait_ms << "ms"
                      << " r=" << prewarmer.radius()
                      << " threads=" << prewarmer.thread_count();
            if (!prewarm_completed || prewarmer.cancelled()) {
                std::cerr << " stopped=prewarm";
            }
            std::cerr << "\n";
        }
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
                if (optsvr::cfg().game_highway_enabled) {
                    uint8_t fb = ORZ_Highway::get_forbid(env->curr_states[a].location);
                    if (fb && fb != 15) pa += 5000;
                    fb = ORZ_Highway::get_forbid(env->curr_states[b].location);
                    if (fb && fb != 15) pb += 5000;
                    if (ORZ_Highway::is_exit_cell(env->curr_states[a].location)) pa = 1e9;
                    if (ORZ_Highway::is_exit_cell(env->curr_states[b].location)) pb = 1e9;
                }
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
            if (optsvr::cfg().game_guide_path_enabled) {
                trajLNS.init_mem();
            }

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

        int step;
        for (step = 0; step < num_steps; step++){
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
        static int step_count = 0;
        int completed_steps = step + 1;
        if (++step_count <= 5)
            std::cerr << "[DIAG] plan() completed " << completed_steps << " steps (num_steps=" << num_steps << ")\n";
    }
}
