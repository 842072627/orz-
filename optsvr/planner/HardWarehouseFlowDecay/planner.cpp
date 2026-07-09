// optsvr/planner/HardWarehouseFlowDecay/planner.cpp
// Source: IDEA/warehouse_new:optsvr/planner/TestUsefulFlowDecay.
// Isolated HARD warehouse FlowDecay planner exposed as planner_mode=13.
// optplan/planner.cpp — 优化版 Planner 实现
//
// 完整复制 DefaultPlanner::plan 的逻辑骨架，
// 维护自己的完整状态（TrajLNS、decision、priority 等）。

#include "planner.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "MapProfile.h"
#include "WarehouseRailConstraints.h"

// HardWarehouseFlowDecayPlan 命名空间下的副本（从 default_planner 复制，可自由修改）
#include "heuristics.h"
#include "flow.h"
#include "fullfill_highway.h"
#include "search.h"
#include "pibt.h"
#include "utils.h"
#include "const.h"
#include "home_return.h"

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <deque>
#include <numeric>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#include <iostream>

using namespace HardWarehouseFlowDecayPlan;

namespace {

int resolve_dist2path_prewarm_threads(int configured_threads) {
    if (configured_threads > 0) return configured_threads;
    const unsigned int hc = std::thread::hardware_concurrency();
    if (hc <= 1) return 1;
    return static_cast<int>(hc - 1);
}

void prewarm_dist_2_path_radius(Dist2Path& dp, const Neighbors* ns, int radius, int cols,
                                const std::atomic<bool>& cancel_requested) {
    if (radius <= 0) return;
    std::vector<int> neighbors;
    while (!dp.open.empty()) {
        if (cancel_requested.load(std::memory_order_relaxed)) return;
        d2p curr = dp.open.front();
        if (curr.cost >= radius) return;
        dp.open.pop_front();
        getNeighborLocs(ns, neighbors, curr.id);
        for (int next_location : neighbors) {
            // Highway forbid check in BFS spread
            uint8_t forbid = HardWarehouseFlowDecayPlan::get_highway_forbid(curr.id);
            if (forbid != 0) {
                int delta = next_location - curr.id;
                int dir = (delta == 1) ? 0 : (delta == -1) ? 2 : (delta == cols) ? 1 : 3;
                bool blocked = false;
                if (dir == 0 && (forbid & 2)) blocked = true;
                if (dir == 1 && (forbid & 8)) blocked = true;
                if (dir == 2 && (forbid & 1)) blocked = true;
                if (dir == 3 && (forbid & 4)) blocked = true;
                if (blocked) continue;
            }
            const int cost = curr.cost + 1;
            auto it_next = dp.dist2path.find(next_location);
            if (it_next != dp.dist2path.end() &&
                it_next->second.label == dp.label &&
                cost >= it_next->second.cost) {
                continue;
            }
            dp.open.emplace_back(dp.label, next_location, cost, curr.togo);
            dp.dist2path.insert_or_assign(next_location,
                                          d2p(dp.label, next_location, cost, curr.togo));
        }
    }
}

class Dist2PathPrewarmer {
public:
    Dist2PathPrewarmer(SharedEnvironment* env, Neighbors* neighbors,
                       int radius, int configured_threads)
        : env_(env),
          neighbors_(neighbors),
          radius_(std::max(0, radius)),
          thread_count_(resolve_dist2path_prewarm_threads(configured_threads)),
          slots_(env != nullptr ? env->num_of_agents : 0) {
        if (env_ == nullptr || neighbors_ == nullptr || radius_ <= 0 || slots_.empty()) {
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

    ~Dist2PathPrewarmer() {
        request_stop(true);
        join_workers();
    }

    Dist2PathPrewarmer(const Dist2PathPrewarmer&) = delete;
    Dist2PathPrewarmer& operator=(const Dist2PathPrewarmer&) = delete;

    bool enabled() const { return enabled_; }
    int radius() const { return radius_; }
    int thread_count() const { return thread_count_; }

    void enqueue(int agent, const Traj& path) {
        if (!enabled_ || path.empty() || agent < 0 || agent >= static_cast<int>(slots_.size())) return;
        std::lock_guard<std::mutex> lk(mu_);
        if (!accepting_) return;
        Slot& slot = slots_[agent];
        slot.latest_path_version++;
        slot.path = path;
        auto old_result = results_.find(agent);
        if (old_result != results_.end() &&
            old_result->second.path2dist_version != slot.latest_path_version) {
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

    bool finish_until(TimePoint deadline) {
        if (!enabled_) return true;
        bool completed = false;
        {
            std::unique_lock<std::mutex> lk(mu_);
            accepting_ = false;
            completed = done_cv_.wait_until(lk, deadline, [&]() {
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

    int commit(TrajLNS& lns) {
        if (!enabled_) return 0;
        int committed = 0;
        for (auto& item : results_) {
            const int agent = item.first;
            if (agent < 0 || agent >= static_cast<int>(lns.traj_dists.size())) continue;
            if (agent >= static_cast<int>(slots_.size())) continue;
            Slot& slot = slots_[agent];
            if (item.second.path2dist_version != slot.latest_path_version ||
                slot.path2dist_version != slot.latest_path_version) {
                discarded_++;
                continue;
            }
            lns.traj_dists[agent] = std::move(item.second.dp);
            committed++;
        }
        results_.clear();
        committed_ += committed;
        return committed;
    }

    int enqueued() const { return enqueued_; }
    int built() const { return built_; }
    int discarded() const { return discarded_; }
    int committed_total() const { return committed_; }
    bool cancelled() const { return cancel_requested_.load(std::memory_order_relaxed); }

private:
    struct Slot {
        int latest_path_version = 0;
        int path2dist_version = 0;
        bool queued = false;
        bool running = false;
        bool dirty = false;
        Traj path;
    };

    struct Result {
        int path2dist_version = 0;
        Dist2Path dp;
    };

    void request_stop(bool cancel) {
        if (!enabled_) return;
        {
            std::lock_guard<std::mutex> lk(mu_);
            accepting_ = false;
            stopping_ = true;
            if (cancel) cancel_requested_.store(true, std::memory_order_relaxed);
        }
        cv_.notify_all();
        done_cv_.notify_all();
    }

    void join_workers() {
        for (auto& worker : workers_) {
            if (worker.joinable()) worker.join();
        }
        workers_.clear();
    }

    void worker_loop() {
        while (true) {
            int agent = -1;
            int job_path_version = 0;
            Traj path;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&]() {
                    return stopping_ || !queue_.empty();
                });
                if (stopping_ && queue_.empty()) return;
                agent = queue_.front();
                queue_.pop_front();
                Slot& slot = slots_[agent];
                slot.queued = false;
                slot.running = true;
                slot.dirty = false;
                job_path_version = slot.latest_path_version;
                path = slot.path;
                active_jobs_++;
            }

            Result result;
            result.path2dist_version = job_path_version;
            if (!path.empty() && !cancel_requested_.load(std::memory_order_relaxed)) {
                init_dist_2_path(result.dp, env_, path);
                prewarm_dist_2_path_radius(result.dp, neighbors_, radius_, env_->cols, cancel_requested_);
            }

            {
                std::lock_guard<std::mutex> lk(mu_);
                active_jobs_--;
                Slot& slot = slots_[agent];
                slot.running = false;
                if (!cancel_requested_.load(std::memory_order_relaxed) &&
                    !slot.dirty && slot.latest_path_version == job_path_version) {
                    slot.path2dist_version = job_path_version;
                    results_[agent] = std::move(result);
                    built_++;
                } else {
                    discarded_++;
                    if (!stopping_ && slot.latest_path_version != job_path_version && !slot.queued) {
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
    Neighbors* neighbors_ = nullptr;
    int radius_ = 0;
    int thread_count_ = 0;
    bool enabled_ = false;

    mutable std::mutex mu_;
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
    int committed_ = 0;
};

}  // namespace

namespace optsvr {

// ============ rollout_next_state ============
State HardWarehouseFlowDecayPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
{
    State next = state;
    next.timestep = state.timestep + 1;

    if (action == Action::CR) {
        next.orientation = (state.orientation + 1) % 4;
        return next;
    }
    if (action == Action::CCR) {
        next.orientation = (state.orientation + 3) % 4;
        return next;
    }
    if (action != Action::FW) {
        return next;
    }

    int delta = 0;
    if (state.orientation == 0) delta = 1;
    if (state.orientation == 1) delta = env->cols;
    if (state.orientation == 2) delta = -1;
    if (state.orientation == 3) delta = -env->cols;

    int next_loc = state.location + delta;
    if (next_loc >= 0 && next_loc < (int)env->map.size() && validateMove(state.location, next_loc, env)) {
        next.location = next_loc;
    }
    return next;
}

// ============ initialize ============
void HardWarehouseFlowDecayPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    assert(env->num_of_agents != 0);

    p_.resize(env->num_of_agents);
    decision_.resize(env->map.size(), -1);
    prev_states_.resize(env->num_of_agents);
    next_states_.resize(env->num_of_agents);
    decided_.resize(env->num_of_agents, DCR({-1, DONE::DONE}));
    occupied_.resize(env->map.size(), false);
    checked_.resize(env->num_of_agents, false);
    ids_.resize(env->num_of_agents);
    require_guide_path_.resize(env->num_of_agents, false);

    for (int i = 0; i < (int)ids_.size(); i++) {
        ids_[i] = i;
    }

    // 初始化全局启发式表（DefaultPlanner 的全局数据）
    init_heuristics(env);
    mt1_.seed(0);
    srand(0);

    // 使用 placement new 初始化 TrajLNS
    new (&trajLNS_) TrajLNS(env, global_heuristictable, global_neighbors);
    trajLNS_.init_mem();

    // 分配初始优先级
    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < (int)ids_.size(); i++) {
        p_[ids_[i]] = ((double)(ids_.size() - i)) / ((double)(ids_.size() + 1));
    }
    p_copy_ = p_;
    initialized_ = true;

    // 初始化 PIBT 密度惩罚系统
    pibt_density_.initialize(env);

    // Fullfill highway 硬约束（从 fullfill_highway.h 加载）
    if (cfg().hard_ware_direct && map_profile().type == MapType::WAREHOUSE_DOMAIN) {
        build_fullfill_highway(env);
    }
}

// ============ setup_multistep_episode_state ============
void HardWarehouseFlowDecayPlanner::setup_multistep_episode_state(
    SharedEnvironment* env, TimePoint flow_end_time,
    std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    // 保存原始起点（DefaultPlanner 方案：独立 flag，不用 curr_timestep）
    static bool g_home_inited = false;
    if (!g_home_inited && env->num_of_agents > 0) {
        g_home_start_locs.resize(env->num_of_agents);
        for (int j = 0; j < env->num_of_agents; j++)
            g_home_start_locs[j] = env->start_states[j].location;
        g_home_inited = true;
    }

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = g_home_start_locs[i];
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
        }

        // 检测 guide-path mismatch / need replan
        require_guide_path_[i] = false;
        if (trajLNS_.trajs[i].empty()) {
            require_guide_path_[i] = true;
        } else if (trajLNS_.trajs[i].back() != trajLNS_.tasks[i]) {
            require_guide_path_[i] = true;
        }

        // 更新 per-agent 状态
        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (decided_[i].loc == -1) {
            decided_[i].loc = env->curr_states[i].location;
            assert(decided_[i].state == DONE::DONE);
        }
        if (prev_states_[i].location == decided_[i].loc) {
            decided_[i].state = DONE::DONE;
        }
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        // 跨 episode 的优先级更新
        if (require_guide_path_[i])
            local_priority[i] = p_copy_[i];
        else if (!env->goal_locations[i].empty()) {
            // 回家任务不递增优先级（保持基线 p_copy_）
            if (!(optsvr::cfg().hard_dummy_goal_home &&
                  i < (int)g_home_start_locs.size() &&
                  trajLNS_.tasks[i] == g_home_start_locs[i]))
                local_priority[i] = local_priority[i] + 1;
        }

        if (!env->goal_locations[i].empty() &&
            trajLNS_.neighbors[env->curr_states[i].location].size() == 1) {
            local_priority[i] = local_priority[i] + 10;
        }

        // Idle relocation: lowest priority
        if (!env->goal_locations[i].empty() &&
            env->goal_locations[i].front().second == -1) {
            local_priority[i] = -1e6;
        }

        // Safety: real tasks must not carry over -1e6 from prior relocation
        if (!env->goal_locations[i].empty() &&
            env->goal_locations[i].front().second != -1 &&
            local_priority[i] < -1e5) {
            local_priority[i] = static_cast<double>(env->num_of_agents - i) /
                                static_cast<double>(env->num_of_agents + 1);
        }
    }
}

// ============ update_guide_paths_once_for_multistep ============
void HardWarehouseFlowDecayPlanner::update_guide_paths_once_for_multistep(SharedEnvironment* env, TimePoint flow_end_time)
{
    // Determine extra_weights pointer: build combined_weights_ = extra_weights_ + flow radiation
    combined_weights_ = extra_weights_;
    // ─── Flow radiation: 8-directional ray-casting, stops at walls ───
    if (!trajLNS_.flow.empty() && env != nullptr && 
        (int)combined_weights_.size() == env->map.size() && env->cols > 0) {
        const int R = env->rows, C = env->cols, M = R * C;
        const int di[8] = {-1,1,0,0,-1,-1,1,1};
        const int dj[8] = {0,0,-1,1,-1,1,-1,1};
        const float kernel[9] = {0.18f,0.16f,0.14f,0.12f,0.10f,0.08f,0.06f,0.04f,0.02f};
        // Precompute total flow per cell
        std::vector<int> heat(M, 0);
        for (int i = 0; i < M; i++)
            for (int k = 0; k < 4; k++) heat[i] += trajLNS_.flow[i].d[k];
        for (int i = 0; i < R; i++) {
            for (int j = 0; j < C; j++) {
                int idx = i * C + j;
                if (!env->map[idx]) {
                    float total = heat[idx] * kernel[0];
                    for (int d = 0; d < 8; d++) {
                        for (int k = 1; k <= 8; k++) {
                            int ni = i + di[d] * k, nj = j + dj[d] * k;
                            if (ni < 0 || ni >= R || nj < 0 || nj >= C) break;
                            int nidx = ni * C + nj;
                            if (env->map[nidx]) break;
                            total += heat[nidx] * kernel[k];
                        }
                    }
                    int final_heat = (int)(total + 0.5f);
                    int w = (heat[idx] + final_heat) / 2 / 600;
                    if (w > 0)
                        for (int d = 0; d < 4; d++)
                            combined_weights_[idx][d] += w;
                }
            }
        }
    }
    const std::vector<std::vector<int>>* ew_ptr = &combined_weights_;
    const int prewarm_radius =
        cfg().dist2path_prewarm_enabled ? cfg().dist2path_prewarm_radius : 0;
    Dist2PathPrewarmer prewarmer(env, &(trajLNS_.neighbors),
                                 prewarm_radius,
                                 cfg().dist2path_prewarm_threads);
    int prewarm_skipped_by_time = 0;
    auto enqueue_prewarm = [&](int agent) {
        const auto left_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            flow_end_time - std::chrono::steady_clock::now()).count();
        if (left_ms < std::max(0, cfg().dist2path_prewarm_min_left_ms)) {
            prewarm_skipped_by_time++;
            return;
        }
        prewarmer.enqueue(agent, trajLNS_.trajs[agent]);
    };

    int required = 0;
    for (bool need_update : require_guide_path_) {
        if (need_update) required++;
    }

    int completed = 0;
    int failed = 0;
    bool stopped_by_time = false;
    const auto flow_phase_start = std::chrono::steady_clock::now();

    // 串行 guide path（与 DefaultPlanner 完全一致）
    for (int i = 0; i < env->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() >= flow_end_time) {
            stopped_by_time = true;
            break;
        }
        if (require_guide_path_[i]) {
            const bool had_old_traj = !trajLNS_.trajs[i].empty();
            const int old_last_replan_t = trajLNS_.fw_metrics[i].last_replan_t;
            if (!trajLNS_.trajs[i].empty())
                remove_traj(trajLNS_, i);
            if (update_traj(trajLNS_, i, &flow_end_time, ew_ptr)) {
                completed++;
                enqueue_prewarm(i);
            } else {
                if (had_old_traj) {
                    add_traj(trajLNS_, i);
                    trajLNS_.fw_metrics[i].last_replan_t = old_last_replan_t;
                }
                failed++;
                if (std::chrono::steady_clock::now() >= flow_end_time) {
                    stopped_by_time = true;
                    break;
                }
            }
        }
    }

    // 串行 Frank-Wolfe
    std::unordered_set<int> updated;
    if (std::chrono::steady_clock::now() < flow_end_time) {
        frank_wolfe(trajLNS_, updated, flow_end_time, ew_ptr, enqueue_prewarm);
    } else {
        stopped_by_time = true;
    }

    const auto prewarm_wait_start = std::chrono::steady_clock::now();
    const bool prewarm_completed = prewarmer.finish_until(flow_end_time);
    const int prewarm_committed = prewarmer.commit(trajLNS_);
    const int prewarm_wait_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now() - prewarm_wait_start).count());

    if (cfg().timing_trace) {
        const int flow_ms = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - flow_phase_start).count());
        const int ready = env->num_of_agents - required + completed;
        std::cerr << "[TIMING][GUIDE]"
                  << " guide=" << completed << "/" << required
                  << " ready=" << ready << "/" << env->num_of_agents
                  << " failed=" << failed
                  << " elapsed=[" << flow_ms << "ms]"
                  << " prewarm_q=" << prewarmer.enqueued()
                  << " prewarm_built=" << prewarmer.built()
                  << " prewarm_commit=" << prewarm_committed
                  << " prewarm_discard=" << prewarmer.discarded()
                  << " prewarm_skip=" << prewarm_skipped_by_time
                  << " prewarm_wait=[" << prewarm_wait_ms << "ms]"
                  << " prewarm_r=" << prewarmer.radius()
                  << " prewarm_threads=" << prewarmer.thread_count();
        if (!prewarm_completed || prewarmer.cancelled()) std::cerr << " stopped=prewarm";
        if (stopped_by_time) std::cerr << " stopped=time";
        std::cerr << "\n";
    }
}

// ============ refresh_multistep_step_state ============
void HardWarehouseFlowDecayPlanner::refresh_multistep_step_state(SharedEnvironment* env, std::vector<double>& local_priority)
{
    prev_decision_.clear();
    prev_decision_.resize(env->map.size(), -1);

    for (int i = 0; i < env->num_of_agents; i++) {
        if (env->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = g_home_start_locs[i];
            local_priority[i] = -1e6;
        } else {
            trajLNS_.tasks[i] = env->goal_locations[i].front().first;
            local_priority[i] = local_priority[i] + 1;
        }

        assert(env->curr_states[i].location >= 0);
        prev_states_[i] = env->curr_states[i];
        next_states_[i] = State();
        prev_decision_[env->curr_states[i].location] = i;

        if (prev_states_[i].location == decided_[i].loc) {
            decided_[i].state = DONE::DONE;
        }
        if (decided_[i].state == DONE::NOT_DONE) {
            decision_.at(decided_[i].loc) = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }

        if (!env->goal_locations[i].empty() &&
            trajLNS_.neighbors[env->curr_states[i].location].size() == 1) {
            local_priority[i] = local_priority[i] + 10;
        }

        // Idle relocation: lowest priority
        if (!env->goal_locations[i].empty() &&
            env->goal_locations[i].front().second == -1) {
            local_priority[i] = -1e6;
        }

        // Safety: real tasks must not carry over -1e6 from prior relocation
        if (!env->goal_locations[i].empty() &&
            env->goal_locations[i].front().second != -1 &&
            local_priority[i] < -1e5) {
            local_priority[i] = static_cast<double>(env->num_of_agents - i) /
                                static_cast<double>(env->num_of_agents + 1);
        }
    }
}

// ============ run_multistep_pibt_once ============
void HardWarehouseFlowDecayPlanner::run_multistep_pibt_once(
    SharedEnvironment* env, std::vector<double>& local_priority,
    std::vector<Action>& one_step_actions)
{
    pibt_density_.record_occupancy(env->curr_states, env->num_of_agents);
    const HardWarehousePibtDensity* history_congestion =
        pibt_density_.active() ? &pibt_density_ : nullptr;

    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
        return local_priority.at(a) > local_priority.at(b);
    });

    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == DONE::NOT_DONE) {
            continue;
        }
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env->map.size());
            causalPIBT(i, -1, prev_states_, next_states_,
                       prev_decision_, decision_,
                       occupied_, trajLNS_, history_congestion);
        }
    }

    one_step_actions.resize(env->num_of_agents);
    for (int id : ids_) {
        if (next_states_.at(id).location != -1)
            decision_.at(next_states_.at(id).location) = -1;
        if (next_states_.at(id).location >= 0) {
            decided_.at(id) = DCR({next_states_.at(id).location, DONE::NOT_DONE});
        }
        one_step_actions.at(id) = getAction(prev_states_.at(id), decided_.at(id).loc, env);
        checked_.at(id) = false;
    }

    for (int id = 0; id < env->num_of_agents; id++) {
        if (!checked_.at(id) && one_step_actions.at(id) == Action::FW) {
            moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
        }
    }

    prev_states_ = next_states_;
}

// ============ append_actions_and_rollout_states ============
void HardWarehouseFlowDecayPlanner::append_actions_and_rollout_states(
    SharedEnvironment* env,
    std::vector<std::vector<Action>>& actions,
    const std::vector<Action>& one_step_actions)
{
    for (int aid = 0; aid < env->num_of_agents; aid++) {
        actions[aid].push_back(one_step_actions[aid]);
    }

    std::vector<State> rolled_states(env->num_of_agents);
    for (int aid = 0; aid < env->num_of_agents; aid++) {
        rolled_states[aid] = rollout_next_state(env->curr_states[aid], one_step_actions[aid], env);
    }
    env->curr_states = rolled_states;
    env->curr_timestep += 1;
}

// ============ plan ============
void HardWarehouseFlowDecayPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                      SharedEnvironment* env, int num_steps)
{
    actions.clear();
    if (env == nullptr || env->num_of_agents <= 0) {
        return;
    }

    actions.resize(env->num_of_agents);
    if (num_steps <= 0) {
        return;
    }

    const auto episode_start = std::chrono::steady_clock::now();

    // 时间预算分配：
    // - 给 PIBT 留出固定时间（实测 5000 agents 单步 PIBT < 50ms）
    // - 剩余全部给 flow 阶段（guide path + frank_wolfe）
    // 原始估算 PIBT_RUNTIME_PER_100_AGENTS=1ms 对 5000 agents 只预留 50ms × num_steps，
    // 但 DefaultPlanner 还保留了 TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE=10ms 的容差
    int pibt_time = PIBT_RUNTIME_PER_100_AGENTS * env->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    const int flow_budget_ms = std::max(0, time_limit - pibt_time * num_steps - TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

    // 硬截止时间：time_limit - 安全余量（确保无论如何都能在截止前返回）
    TimePoint hard_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit - 5));

    std::vector<double> local_priority = p_;

    const std::vector<State> original_states = env->curr_states;
    const int original_timestep = env->curr_timestep;
    const std::vector<DCR> original_decided = decided_;

    decided_.assign(env->num_of_agents, DCR({-1, DONE::DONE}));

    setup_multistep_episode_state(env, flow_end_time, local_priority);

    update_guide_paths_once_for_multistep(env, flow_end_time);

    // commit cross-episode priority
    p_ = local_priority;

    for (int step = 0; step < num_steps; step++) {
        // 超时保护：如果接近硬截止时间，提前终止 PIBT 循环
        // 至少完成 1 步以保证有合法动作输出
        if (step > 0 && std::chrono::steady_clock::now() >= hard_deadline) {
            break;
        }

        std::vector<Action> one_step_actions;

        if (step > 0) {
            refresh_multistep_step_state(env, local_priority);
        }

        run_multistep_pibt_once(env, local_priority, one_step_actions);
        append_actions_and_rollout_states(env, actions, one_step_actions);

        // 保存每步动作，最后一步的将用于下一轮密度统计更新
        last_step_actions_ = one_step_actions;
        last_step_states_ = env->curr_states;  // rollout 后的状态
    }

    // 恢复 env state
    env->curr_states = original_states;
    env->curr_timestep = original_timestep;
    decided_ = original_decided;
}

// ============ build_fullfill_highway (runtime computation) ============
// Compute per-cell direction constraints from map structure.
// Algorithm: detect passage rows/columns, apply alternating direction forbids,
// add wall-adjacent checks, and generate guide zone penalties at boundaries.
// Same logic as the HTML editor's generateInitialConstraints().
void HardWarehouseFlowDecayPlanner::build_fullfill_highway(SharedEnvironment* env)
{
    auto stats = optsvr::build_warehouse_planner_rail_constraints(
        env, nullptr, &extra_weights_);
    const int map_size = env ? static_cast<int>(env->map.size()) : 0;

    // Initialize forbid mask for BFS and PIBT (derived from extra_weights_)
    HardWarehouseFlowDecayPlan::init_highway_forbid_mask_from_weights(extra_weights_, map_size);

    if (cfg().debug_level > 0) {
        std::cerr << "[HardWarehouseFlowDecayPlanner] FULLFILL highway: "
                  << stats.forbidden_edges << " forbids, cost=" << WAREHOUSE_RAIL_FORBID_COST
                  << ", guide_penalty=" << WAREHOUSE_RAIL_GUIDE_PENALTY
                  << ", rows=" << stats.passage_rows << " cols=" << stats.passage_cols << "\n";
    }
}

} // namespace optsvr
