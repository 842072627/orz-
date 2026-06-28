// optsvr/planner/MazeTidalPlanner/MazeTidalPlanner.cpp
// Source: IDEA/MAZEOpt MAPFPlanner maze_wall_follow_vote_tidal_highway.
#include "MazeTidalPlanner.h"

#include <algorithm>
#include <cassert>
#include <chrono>
#include <climits>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace optsvr {
namespace {

constexpr int kMazeBottomVoteMinRow = 25;
constexpr int kMazeBottomVoteMaxRow = 31;
constexpr int kMazeBottomVoteMinCol = 7;
constexpr int kMazeBottomVoteMaxCol = 31;
constexpr int kMazeBottomVoteExcludeLeftMinRow = 25;
constexpr int kMazeBottomVoteExcludeLeftMaxRow = 31;
constexpr int kMazeBottomVoteExcludeLeftMinCol = 7;
constexpr int kMazeBottomVoteExcludeLeftMaxCol = 9;
constexpr int kMazeBottomVoteExcludeRightMinRow = 25;
constexpr int kMazeBottomVoteExcludeRightMaxRow = 31;
constexpr int kMazeBottomVoteExcludeRightMinCol = 29;
constexpr int kMazeBottomVoteExcludeRightMaxCol = 31;
constexpr int kMazeBottomLeftCol = 16;
constexpr int kMazeBottomRightCol = 17;
constexpr int kMazeBottomDoorRow = 31;
constexpr int kMazeBottomWestDoorCol = 15;
constexpr int kMazeBottomEastDoorCol = 18;
constexpr int kMazeBottomThirdDoorCol = 21;
constexpr int kMazeBottomEastRefugeCol = 22;
constexpr int kMazeBottomEastRefugeMinRow = 28;
constexpr int kMazeBottomEastRefugeMaxRow = 31;
constexpr int kMazeRightHandHighwayPenalty = 240;
constexpr int kMazeCentralBottomSplitPenalty = 240;
constexpr int kMazeRightHandOffwallPenalty = 1;
constexpr int kMazeBottomPriorityPeriod = 80;
constexpr double kMazeBottomTidalPriorityDelta = 60.0;
constexpr double kMazeBottomCol17FrontPriorityStep = 20.0;
constexpr double kMazeBottomExitReversePriorityBoost = 200000.0;
constexpr int kRoomGuideNextStepBonus = 30;
constexpr int kRoomGuideOffPathPenalty = 45;
constexpr int kRoomGuideBacktrackPenalty = 45;
constexpr int kRoomGuideBacktrackStepPenalty = 8;
constexpr int kRoomGuideWaitPenalty = 8;
constexpr int kRoomGuideForwardSkipPenalty = 4;

bool maze_bottom_door_contains(int row, int col)
{
    return row == kMazeBottomDoorRow &&
           (col == kMazeBottomWestDoorCol ||
            col == kMazeBottomEastDoorCol ||
            col == kMazeBottomThirdDoorCol);
}

bool maze_bottom_vote_excluded_contains(int row, int col)
{
    const bool in_left =
        row >= kMazeBottomVoteExcludeLeftMinRow &&
        row <= kMazeBottomVoteExcludeLeftMaxRow &&
        col >= kMazeBottomVoteExcludeLeftMinCol &&
        col <= kMazeBottomVoteExcludeLeftMaxCol;
    const bool in_right =
        row >= kMazeBottomVoteExcludeRightMinRow &&
        row <= kMazeBottomVoteExcludeRightMaxRow &&
        col >= kMazeBottomVoteExcludeRightMinCol &&
        col <= kMazeBottomVoteExcludeRightMaxCol;
    return in_left || in_right;
}

bool maze_bottom_vote_buffer_contains(int row, int col)
{
    return row >= kMazeBottomVoteMinRow && row <= kMazeBottomVoteMaxRow &&
           col >= kMazeBottomVoteMinCol && col <= kMazeBottomVoteMaxCol &&
           !maze_bottom_vote_excluded_contains(row, col);
}

bool maze_bottom_parking_contains(int row, int col)
{
    const bool east_top = row == 28 && col >= 19 && col <= 25;
    const bool east_bottom = row == 29 && col >= 19 && col <= 22;
    const bool east_lower_single = row == 30 && col == 22;
    return east_top || east_bottom || east_lower_single;
}

const std::vector<std::pair<int, int>>& maze_bottom_wander_loop_cells()
{
    static const std::vector<std::pair<int, int>> loop = {
        {26, 16}, {27, 16}, {28, 16}, {29, 16},
        {29, 17}, {28, 17}, {27, 17}, {26, 17},
        {26, 18}, {26, 19}, {26, 20}, {25, 20},
        {25, 19}, {25, 18}, {25, 17}, {25, 16},
    };
    return loop;
}

int maze_bottom_wander_loop_index(int row, int col)
{
    const auto& loop = maze_bottom_wander_loop_cells();
    for (int i = 0; i < static_cast<int>(loop.size()); i++) {
        if (loop[i].first == row && loop[i].second == col) return i;
    }
    return -1;
}

bool maze_bottom_wander_loop_contains(int row, int col)
{
    return maze_bottom_wander_loop_index(row, col) >= 0;
}

int maze_bottom_wander_loop_next_loc(const SharedEnvironment* env, int loc)
{
    if (!env || env->cols <= 0 || loc < 0 || loc >= static_cast<int>(env->map.size()))
        return -1;
    const int row = loc / env->cols;
    const int col = loc % env->cols;
    const int index = maze_bottom_wander_loop_index(row, col);
    if (index < 0) return -1;
    const auto& loop = maze_bottom_wander_loop_cells();
    const auto& next = loop[(index + 1) % loop.size()];
    const int next_loc = next.first * env->cols + next.second;
    if (next_loc < 0 || next_loc >= static_cast<int>(env->map.size()) ||
        env->map[next_loc] != 0)
        return -1;
    return next_loc;
}

int maze_bottom_path_crossing_direction(const std::vector<int>& path, int cols, int* vote_loc = nullptr)
{
    if (vote_loc) *vote_loc = -1;
    if (cols <= 0 || path.size() < 2) return 0;
    for (size_t i = 1; i < path.size(); i++) {
        const int prev = path[i - 1];
        const int curr = path[i];
        if (prev < 0 || curr < 0) continue;
        const int prev_row = prev / cols;
        const int prev_col = prev % cols;
        const int curr_row = curr / cols;
        const int curr_col = curr % cols;
        if (prev_row != curr_row || prev_col == curr_col) continue;
        if (!maze_bottom_door_contains(prev_row, prev_col) &&
            !maze_bottom_door_contains(curr_row, curr_col))
            continue;
        if (curr_col > prev_col) {
            if (vote_loc) *vote_loc = maze_bottom_door_contains(prev_row, prev_col) ? prev : curr;
            return 1;
        }
        if (curr_col < prev_col) {
            if (vote_loc) *vote_loc = maze_bottom_door_contains(prev_row, prev_col) ? prev : curr;
            return -1;
        }
    }
    return 0;
}

int maze_bottom_vote_group_from_loc(int loc, int cols)
{
    if (loc < 0 || cols <= 0) return 0;
    const int row = loc / cols;
    const int col = loc % cols;
    if (row != kMazeBottomDoorRow) return 0;
    if (col == kMazeBottomEastDoorCol || col == kMazeBottomThirdDoorCol) return 2;
    return 0;
}

void apply_maze_key_cell_priority_boost(const SharedEnvironment* env,
                                        std::vector<double>& pibt_priority)
{
    if (!env || env->rows != 32 || env->cols != 32 ||
        static_cast<int>(pibt_priority.size()) < env->num_of_agents)
        return;
    constexpr int key_row = 26;
    constexpr int key_col = 16;
    const int key_loc = key_row * env->cols + key_col;
    for (int aid = 0; aid < env->num_of_agents; ++aid) {
        if (aid < static_cast<int>(env->start_states.size()) &&
            env->start_states[aid].location == key_loc)
            pibt_priority[aid] += 1000.0;
    }
}

struct GuideSequencePenalty {
    int penalty = 0;
    int current_index = -1;
    int candidate_index = -1;
    int expected_index = -1;
    bool candidate_on_path = false;
    bool expected_next_step = false;
};

int guide_path_closest_index(const std::vector<int>& path, int loc, int cols, bool* exact = nullptr)
{
    if (exact) *exact = false;
    if (path.empty() || loc < 0 || cols <= 0) return -1;
    for (int i = 0; i < static_cast<int>(path.size()); i++) {
        if (path[i] == loc) {
            if (exact) *exact = true;
            return i;
        }
    }
    const int row = loc / cols;
    const int col = loc % cols;
    int best_index = 0;
    int best_dist = std::numeric_limits<int>::max();
    for (int i = 0; i < static_cast<int>(path.size()); i++) {
        const int path_loc = path[i];
        if (path_loc < 0) continue;
        const int dist = std::abs(path_loc / cols - row) + std::abs(path_loc % cols - col);
        if (dist < best_dist) {
            best_dist = dist;
            best_index = i;
        }
    }
    return best_index;
}

GuideSequencePenalty get_full_guide_sequence_penalty(const MazePlan::TrajLNS& lns,
                                                     int agent_id,
                                                     int prev_loc,
                                                     int candidate_loc)
{
    GuideSequencePenalty result;
    if (agent_id < 0 || agent_id >= static_cast<int>(lns.trajs.size()) ||
        !lns.env || lns.env->cols <= 0)
        return result;

    const std::vector<int>& path = lns.trajs[agent_id];
    if (path.size() < 2) return result;

    result.current_index = guide_path_closest_index(path, prev_loc, lns.env->cols);
    bool candidate_exact = false;
    result.candidate_index =
        guide_path_closest_index(path, candidate_loc, lns.env->cols, &candidate_exact);
    result.candidate_on_path = candidate_exact;
    if (result.current_index < 0 || result.candidate_index < 0) return result;

    result.expected_index = std::min(result.current_index + 1, static_cast<int>(path.size()) - 1);
    result.expected_next_step = candidate_loc == path[result.expected_index];
    if (result.expected_next_step) {
        result.penalty -= kRoomGuideNextStepBonus;
        return result;
    }
    if (candidate_loc == prev_loc) {
        result.penalty += kRoomGuideWaitPenalty;
        return result;
    }
    if (!candidate_exact) result.penalty += kRoomGuideOffPathPenalty;
    if (result.candidate_index < result.current_index) {
        result.penalty += kRoomGuideBacktrackPenalty +
                          kRoomGuideBacktrackStepPenalty *
                              (result.current_index - result.candidate_index);
    } else if (result.candidate_index > result.expected_index) {
        result.penalty += kRoomGuideForwardSkipPenalty *
                          (result.candidate_index - result.expected_index);
    }
    return result;
}

int get_extra_pibt_weight(const std::vector<std::vector<int>>& extra_weights,
                          const SharedEnvironment* env,
                          int from,
                          int to)
{
    if (from < 0 || to < 0 || from >= static_cast<int>(extra_weights.size()))
        return 0;
    const int d = MazePlan::get_d(to - from, env);
    if (d < 0 || d >= static_cast<int>(extra_weights[from].size())) return 0;
    return extra_weights[from][d];
}

} // namespace

void MazeTidalPlanner::initialize(int /*preprocess_time_limit*/, SharedEnvironment* env)
{
    env_ = env;
    assert(env_ && env_->num_of_agents != 0);

    priorities_.resize(env_->num_of_agents);
    decision_.resize(env_->map.size(), -1);
    prev_states_.resize(env_->num_of_agents);
    next_states_.resize(env_->num_of_agents);
    decided_.resize(env_->num_of_agents, MazePlan::DCR({-1, MazePlan::DONE::DONE}));
    occupied_.resize(env_->map.size(), false);
    checked_.resize(env_->num_of_agents, false);
    ids_.resize(env_->num_of_agents);
    require_guide_path_.resize(env_->num_of_agents, false);
    maze_bottom_locked_agent_votes_.assign(env_->num_of_agents, 9);
    maze_bottom_locked_agent_vote_locs_.assign(env_->num_of_agents, -1);
    maze_bottom_locked_agent_vote_groups_.assign(env_->num_of_agents, 0);
    for (int i = 0; i < static_cast<int>(ids_.size()); i++) ids_[i] = i;

    initialize_extra_weights();
    if (is_maze_map()) build_maze_wall_follow_highway();

    mt1_.seed(0);
    srand(0);

    MazePlan::init_heuristics(env_);
    new (&trajLNS_) MazePlan::TrajLNS(env_, MazePlan::global_heuristictable, MazePlan::global_neighbors);
    trajLNS_.init_mem();

    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < static_cast<int>(ids_.size()); i++)
        priorities_[ids_[i]] = static_cast<double>(ids_.size() - i) /
                               static_cast<double>(ids_.size() + 1);
    initial_priorities_ = priorities_;
    initialized_ = true;
    std::cerr << "[MazeTidalPlanner] initialized\n";
}

void MazeTidalPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                            SharedEnvironment* env, int num_steps)
{
    env_ = env;
    actions.clear();
    if (!env_ || env_->num_of_agents <= 0) return;
    actions.resize(env_->num_of_agents);
    if (num_steps <= 0) return;

    const auto episode_start = std::chrono::steady_clock::now();
    const TimePoint episode_deadline =
        episode_start + std::chrono::milliseconds(std::max(0, time_limit));
    int pibt_time = MazePlan::PIBT_RUNTIME_PER_100_AGENTS * env_->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    const int flow_budget_ms = std::max(
        0,
        time_limit - std::max(MazePlan::MIN_PIBT_TIME, pibt_time * num_steps) -
            MazePlan::TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    const TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

    std::vector<double> local_priority = priorities_;
    const std::vector<State> original_states = env_->start_states;
    const int original_curr_timestep = env_->curr_timestep;
    const int original_system_timestep = env_->system_timestep;
    const int original_timestep =
        env_->system_timestep >= 0 ? env_->system_timestep : env_->curr_timestep;
    const std::vector<MazePlan::DCR> original_decided = decided_;

    decided_.assign(env_->num_of_agents, MazePlan::DCR({-1, MazePlan::DONE::DONE}));
    initialize_dummy_goals_if_needed();
    setup_multistep_episode_state(flow_end_time, local_priority);
    update_guide_paths_once(flow_end_time);
    priorities_ = local_priority;

    for (int step = 0; step < num_steps; step++) {
        if (std::chrono::steady_clock::now() >= episode_deadline) break;
        std::vector<Action> one_step_actions;
        if (step > 0) refresh_multistep_step_state(local_priority);
        env_->system_timestep = original_timestep + step;
        run_tidal_pibt_once(local_priority, one_step_actions);
        append_actions_and_rollout_states(actions, one_step_actions);
    }

    env_->start_states = original_states;
    env_->curr_timestep = original_curr_timestep;
    env_->system_timestep = original_system_timestep;
    decided_ = original_decided;
}

bool MazeTidalPlanner::is_maze_map() const
{
    if (!env_ || env_->rows != 32 || env_->cols != 32) return false;
    int obstacle_count = 0;
    for (int cell : env_->map) {
        if (cell != 0) obstacle_count++;
    }
    return obstacle_count >= 300 && obstacle_count <= 420;
}

void MazeTidalPlanner::initialize_extra_weights()
{
    if (!env_) return;
    extra_weights_.assign(env_->rows * env_->cols, std::vector<int>(4, 0));
}

void MazeTidalPlanner::build_maze_wall_follow_highway()
{
    if (!env_ || !is_maze_map()) return;
    if (extra_weights_.size() != static_cast<size_t>(env_->rows * env_->cols))
        initialize_extra_weights();

    const int rows = env_->rows;
    const int cols = env_->cols;
    const int map_size = rows * cols;
    const int d_row[4] = {0, 1, 0, -1};
    const int d_col[4] = {1, 0, -1, 0};

    auto is_walkable = [&](int row, int col) -> bool {
        if (row < 0 || row >= rows || col < 0 || col >= cols) return false;
        return env_->map[row * cols + col] == 0;
    };

    int start_loc = -1;
    for (int row = 0; row < rows && start_loc == -1; row++) {
        for (int col = 0; col < cols; col++) {
            if (is_walkable(row, col)) {
                start_loc = row * cols + col;
                break;
            }
        }
    }
    if (start_loc == -1) return;

    int curr_loc = start_loc;
    int curr_dir = 1;
    std::vector<int> wall_follow_path;
    wall_follow_path.reserve(map_size * 2);
    wall_follow_path.push_back(curr_loc);
    const int max_steps = std::max(1, map_size * 32);
    bool returned_to_start = false;
    for (int step = 0; step < max_steps; step++) {
        const int row = curr_loc / cols;
        const int col = curr_loc % cols;
        int next_loc = -1;
        int next_dir = curr_dir;
        const int candidate_dirs[4] = {
            (curr_dir + 1) % 4,
            curr_dir,
            (curr_dir + 3) % 4,
            (curr_dir + 2) % 4,
        };
        for (int dir : candidate_dirs) {
            const int next_row = row + d_row[dir];
            const int next_col = col + d_col[dir];
            if (is_walkable(next_row, next_col)) {
                next_loc = next_row * cols + next_col;
                next_dir = dir;
                break;
            }
        }
        if (next_loc == -1) break;
        curr_loc = next_loc;
        curr_dir = next_dir;
        wall_follow_path.push_back(curr_loc);
        if (curr_loc == start_loc) {
            returned_to_start = true;
            break;
        }
    }

    std::unordered_map<long long, int> directed_edge_balance;
    directed_edge_balance.reserve(wall_follow_path.size());
    auto edge_key = [](int a, int b) -> long long {
        const int low = std::min(a, b);
        const int high = std::max(a, b);
        return (static_cast<long long>(low) << 32) | static_cast<unsigned int>(high);
    };
    for (int i = 1; i < static_cast<int>(wall_follow_path.size()); i++) {
        const int from = wall_follow_path[i - 1];
        const int to = wall_follow_path[i];
        if (from == to || !MazePlan::validateMove(from, to, env_)) continue;
        directed_edge_balance[edge_key(from, to)] += (from < to) ? 1 : -1;
    }

    int directed_edges = 0;
    int penalized_moves = 0;
    int right_hand_offwall_penalties = 0;
    auto has_right_hand_wall_after_move = [&](int next_loc, int dir) {
        const int next_row = next_loc / cols;
        const int next_col = next_loc % cols;
        const int right_dir = (dir + 1) % 4;
        return !is_walkable(next_row + d_row[right_dir], next_col + d_col[right_dir]);
    };

    if (kMazeRightHandOffwallPenalty > 0) {
        for (int from = 0; from < map_size; from++) {
            if (env_->map[from] != 0) continue;
            const int from_row = from / cols;
            const int from_col = from % cols;
            for (int dir = 0; dir < 4; dir++) {
                const int next_row = from_row + d_row[dir];
                const int next_col = from_col + d_col[dir];
                if (!is_walkable(next_row, next_col)) continue;
                const int next = next_row * cols + next_col;
                if (has_right_hand_wall_after_move(next, dir)) continue;
                extra_weights_[from][dir] =
                    std::max(extra_weights_[from][dir], kMazeRightHandOffwallPenalty);
                right_hand_offwall_penalties++;
            }
        }
    }

    for (const auto& [key, balance] : directed_edge_balance) {
        if (balance == 0) continue;
        const int low = static_cast<int>(key >> 32);
        const int high = static_cast<int>(key & 0xffffffffLL);
        const int preferred_from = balance > 0 ? low : high;
        const int preferred_to = balance > 0 ? high : low;
        const int reverse_from = preferred_to;
        const int reverse_to = preferred_from;
        const int reverse_dir = MazePlan::get_d(reverse_to - reverse_from, env_);
        const int reverse_from_row = reverse_from / cols;
        const int reverse_from_col = reverse_from % cols;
        const int reverse_to_row = reverse_to / cols;
        const int reverse_to_col = reverse_to % cols;
        const bool maze_bottom_middle_door_bypass =
            reverse_from_row == 31 && reverse_to_row == 31 &&
            ((reverse_from_col == 17 && reverse_to_col == 16) ||
             (reverse_from_col == 16 && reverse_to_col == 17));
        const bool maze_bottom_left_corner_bypass =
            reverse_from_row == 31 && reverse_to_row == 31 &&
            reverse_from_col == 14 && reverse_to_col == 13;
        if (maze_bottom_middle_door_bypass || maze_bottom_left_corner_bypass) continue;
        if (reverse_from >= 0 && reverse_from < map_size && reverse_dir >= 0 && reverse_dir < 4) {
            extra_weights_[reverse_from][reverse_dir] =
                std::max(extra_weights_[reverse_from][reverse_dir],
                         kMazeRightHandHighwayPenalty);
            penalized_moves++;
        }
        directed_edges++;
    }

    int central_bottom_split_penalties = 0;
    auto add_split_penalty = [&](int row, int from_col, int to_col) {
        if (!is_walkable(row, from_col) || !is_walkable(row, to_col)) return;
        const int from_loc = row * cols + from_col;
        const int to_loc = row * cols + to_col;
        if (!MazePlan::validateMove(from_loc, to_loc, env_)) return;
        const int dir = MazePlan::get_d(to_loc - from_loc, env_);
        if (dir >= 0 && dir < 4) {
            extra_weights_[from_loc][dir] =
                std::max(extra_weights_[from_loc][dir], kMazeCentralBottomSplitPenalty);
            central_bottom_split_penalties++;
        }
    };
    auto add_split_both = [&](int row, int a, int b) {
        add_split_penalty(row, a, b);
        add_split_penalty(row, b, a);
    };
    auto add_vertical_split = [&](int from_row, int to_row, int col) {
        if (!is_walkable(from_row, col) || !is_walkable(to_row, col)) return;
        const int from_loc = from_row * cols + col;
        const int to_loc = to_row * cols + col;
        if (!MazePlan::validateMove(from_loc, to_loc, env_)) return;
        const int dir = MazePlan::get_d(to_loc - from_loc, env_);
        if (dir >= 0 && dir < 4) {
            extra_weights_[from_loc][dir] =
                std::max(extra_weights_[from_loc][dir], kMazeCentralBottomSplitPenalty);
            central_bottom_split_penalties++;
        }
    };
    for (int row = 27; row <= 29; row++) add_split_both(row, 16, 17);
    add_vertical_split(26, 25, 15);
    add_vertical_split(26, 25, 16);

    std::cerr << "[MazeTidalPlanner] wall-follow highway path_steps="
              << std::max(0, static_cast<int>(wall_follow_path.size()) - 1)
              << " returned=" << (returned_to_start ? 1 : 0)
              << " directed_edges=" << directed_edges
              << " penalized_moves=" << penalized_moves
              << " offwall=" << right_hand_offwall_penalties
              << " split=" << central_bottom_split_penalties << "\n";
}

bool MazeTidalPlanner::update_maze_guide_traj(int agent_id, const TimePoint* deadline)
{
    if (agent_id < 0 || agent_id >= static_cast<int>(trajLNS_.trajs.size()))
        return false;
    return MazePlan::update_traj(trajLNS_, agent_id, deadline, &extra_weights_);
}

void MazeTidalPlanner::frank_wolfe_maze_guide(TimePoint timelimit)
{
    MazePlan::update_fw_metrics(trajLNS_);
    std::vector<MazePlan::FW_Metric> replan_order = trajLNS_.fw_metrics;
    std::sort(replan_order.begin(), replan_order.end(),
        [](MazePlan::FW_Metric& a, MazePlan::FW_Metric& b) {
            if (a.deviation != b.deviation) return a.deviation > b.deviation;
            if (a.last_replan_t != b.last_replan_t) return a.last_replan_t < b.last_replan_t;
            return a.rand > b.rand;
        });

    int count = 0;
    while (std::chrono::steady_clock::now() < timelimit && env_->num_of_agents > 0) {
        const int index = count % env_->num_of_agents;
        const int agent_id = replan_order[index].id;
        count++;
        if (trajLNS_.traj_dists[agent_id].empty() || trajLNS_.trajs[agent_id].empty())
            continue;
        std::vector<int> old_traj = trajLNS_.trajs[agent_id];
        MazePlan::s_node old_goal_node = trajLNS_.goal_nodes[agent_id];
        MazePlan::remove_traj(trajLNS_, agent_id);
        if (!update_maze_guide_traj(agent_id, &timelimit)) {
            trajLNS_.trajs[agent_id] = std::move(old_traj);
            trajLNS_.goal_nodes[agent_id] = old_goal_node;
            MazePlan::add_traj(trajLNS_, agent_id);
            MazePlan::update_dist_2_path(trajLNS_, agent_id);
            break;
        }
    }
}

State MazeTidalPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
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
    if (action != Action::FW) return next;
    int delta = 0;
    if (state.orientation == 0) delta = 1;
    if (state.orientation == 1) delta = env->cols;
    if (state.orientation == 2) delta = -1;
    if (state.orientation == 3) delta = -env->cols;
    const int next_loc = state.location + delta;
    if (next_loc >= 0 && next_loc < static_cast<int>(env->map.size()) &&
        MazePlan::validateMove(state.location, next_loc, env))
        next.location = next_loc;
    return next;
}

void MazeTidalPlanner::initialize_dummy_goals_if_needed()
{
    if (dummy_goals_.size() != static_cast<size_t>(env_->num_of_agents) ||
        env_->curr_timestep == 0) {
        dummy_goals_.resize(env_->num_of_agents);
        for (int i = 0; i < env_->num_of_agents; i++)
            dummy_goals_[i] = env_->start_states[i].location;
    }
}

void MazeTidalPlanner::setup_multistep_episode_state(TimePoint flow_end_time,
                                                     std::vector<double>& local_priority)
{
    prev_decision_.assign(env_->map.size(), -1);
    std::fill(decision_.begin(), decision_.end(), -1);
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() < flow_end_time) {
            for (auto& [goal_loc, _] : env_->goal_locations[i]) {
                if (trajLNS_.heuristics.find(goal_loc) == trajLNS_.heuristics.end() ||
                    trajLNS_.heuristics[goal_loc].empty())
                    MazePlan::init_heuristic(trajLNS_.heuristics[goal_loc], env_, goal_loc);
            }
        }
        if (env_->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = dummy_goals_[i];
            local_priority[i] = initial_priorities_[i];
        } else {
            trajLNS_.tasks[i] = env_->goal_locations[i].front().first;
        }
        require_guide_path_[i] =
            trajLNS_.trajs[i].empty() || trajLNS_.trajs[i].back() != trajLNS_.tasks[i];
        assert(env_->start_states[i].location >= 0);
        prev_states_[i] = env_->start_states[i];
        next_states_[i] = State();
        prev_decision_[env_->start_states[i].location] = i;
        if (decided_[i].loc == -1) decided_[i].loc = env_->start_states[i].location;
        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = MazePlan::DONE::DONE;
        if (decided_[i].state == MazePlan::DONE::NOT_DONE) {
            decision_[decided_[i].loc] = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }
        const bool reached_current_task =
            !env_->goal_locations[i].empty() && env_->start_states[i].location == trajLNS_.tasks[i];
        if (reached_current_task) local_priority[i] = 0.0;
        else if (require_guide_path_[i]) local_priority[i] = initial_priorities_[i];
        else if (!env_->goal_locations[i].empty()) local_priority[i] += 1;
        if (!env_->goal_locations[i].empty() &&
            trajLNS_.neighbors[env_->start_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void MazeTidalPlanner::update_guide_paths_once(TimePoint flow_end_time)
{
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end_time) break;
        if (!require_guide_path_[i]) continue;
        std::vector<int> old_traj = trajLNS_.trajs[i];
        MazePlan::s_node old_goal_node = trajLNS_.goal_nodes[i];
        if (!trajLNS_.trajs[i].empty()) MazePlan::remove_traj(trajLNS_, i);
        if (!update_maze_guide_traj(i, &flow_end_time)) {
            trajLNS_.trajs[i] = std::move(old_traj);
            trajLNS_.goal_nodes[i] = old_goal_node;
            if (!trajLNS_.trajs[i].empty()) {
                MazePlan::add_traj(trajLNS_, i);
                MazePlan::update_dist_2_path(trajLNS_, i);
            }
            break;
        }
    }
    frank_wolfe_maze_guide(flow_end_time);
}

void MazeTidalPlanner::refresh_multistep_step_state(std::vector<double>& local_priority)
{
    prev_decision_.assign(env_->map.size(), -1);
    std::fill(decision_.begin(), decision_.end(), -1);
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (env_->goal_locations[i].empty()) {
            trajLNS_.tasks[i] = dummy_goals_[i];
            local_priority[i] = initial_priorities_[i];
        } else {
            trajLNS_.tasks[i] = env_->goal_locations[i].front().first;
            if (env_->start_states[i].location == trajLNS_.tasks[i])
                local_priority[i] = 0.0;
            else
                local_priority[i] += 1;
        }
        assert(env_->start_states[i].location >= 0);
        prev_states_[i] = env_->start_states[i];
        next_states_[i] = State();
        prev_decision_[env_->start_states[i].location] = i;
        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = MazePlan::DONE::DONE;
        if (decided_[i].state == MazePlan::DONE::NOT_DONE) {
            decision_[decided_[i].loc] = i;
            next_states_[i] = State(decided_[i].loc, -1, -1);
        }
        if (!env_->goal_locations[i].empty() &&
            trajLNS_.neighbors[env_->start_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void MazeTidalPlanner::run_tidal_pibt_once(std::vector<double>& local_priority,
                                           std::vector<Action>& one_step_actions)
{
    std::vector<double> pibt_priority = local_priority;
    apply_maze_key_cell_priority_boost(env_, pibt_priority);
    apply_maze_bottom_vote_tidal_priority(pibt_priority);

    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) {
        return pibt_priority.at(a) > pibt_priority.at(b);
    });
    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == MazePlan::DONE::NOT_DONE) continue;
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 &&
                   prev_states_[i].location < static_cast<int>(env_->map.size()));
            causal_pibt_with_extra_weights(i, -1, prev_states_, next_states_, prev_decision_,
                                           decision_, occupied_, trajLNS_);
        }
    }

    one_step_actions.resize(env_->num_of_agents);
    std::fill(checked_.begin(), checked_.end(), false);
    for (int id : ids_) {
        if (next_states_[id].location != -1) decision_[next_states_[id].location] = -1;
        if (next_states_[id].location >= 0)
            decided_[id] = MazePlan::DCR({next_states_[id].location, MazePlan::DONE::NOT_DONE});
        one_step_actions[id] = MazePlan::getAction(prev_states_[id], decided_[id].loc, env_);
    }
    for (int id = 0; id < env_->num_of_agents; id++) {
        if (!checked_[id] && one_step_actions[id] == Action::FW)
            MazePlan::moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
    }
    prev_states_ = next_states_;
}

void MazeTidalPlanner::append_actions_and_rollout_states(
    std::vector<std::vector<Action>>& actions,
    const std::vector<Action>& one_step_actions)
{
    for (int aid = 0; aid < env_->num_of_agents; aid++)
        actions[aid].push_back(one_step_actions[aid]);
    std::vector<State> rolled(env_->num_of_agents);
    for (int aid = 0; aid < env_->num_of_agents; aid++)
        rolled[aid] = rollout_next_state(env_->start_states[aid], one_step_actions[aid], env_);
    env_->start_states = rolled;
    env_->curr_timestep += 1;
}

int MazeTidalPlanner::maze_bottom_vote_direction(int agent_id) const
{
    if (!is_maze_map() || !env_ || agent_id < 0 || agent_id >= env_->num_of_agents)
        return 0;
    if (agent_id < static_cast<int>(trajLNS_.trajs.size())) {
        const int direction = maze_bottom_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols);
        if (direction != 0) return direction;
    }
    return 0;
}

int MazeTidalPlanner::maze_bottom_vote_group(int agent_id) const
{
    if (!is_maze_map() || !env_ || agent_id < 0 || agent_id >= env_->num_of_agents ||
        agent_id >= static_cast<int>(trajLNS_.trajs.size()))
        return 0;
    int vote_loc = -1;
    const int direction =
        maze_bottom_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols, &vote_loc);
    if (direction == 0) return 0;
    return maze_bottom_vote_group_from_loc(vote_loc, env_->cols);
}

int MazeTidalPlanner::maze_bottom_periodic_preferred_direction(int vote_group) const
{
    if (!is_maze_map() || !env_ || vote_group != 2) return 0;
    const int timestep = env_->system_timestep >= 0 ? env_->system_timestep : env_->curr_timestep;
    const int phase_start = (timestep / kMazeBottomPriorityPeriod) * kMazeBottomPriorityPeriod;
    if (maze_bottom_east_priority_phase_start_ == phase_start)
        return maze_bottom_east_preferred_direction_;

    int right_count = 0;
    int left_count = 0;
    for (int aid = 0; aid < env_->num_of_agents; aid++) {
        if (aid >= static_cast<int>(env_->start_states.size())) continue;
        const int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        const int row = loc / env_->cols;
        const int col = loc % env_->cols;
        if (!maze_bottom_vote_buffer_contains(row, col)) continue;

        int vote_loc = loc;
        int direction = 0;
        if (aid < static_cast<int>(trajLNS_.trajs.size()))
            direction = maze_bottom_path_crossing_direction(trajLNS_.trajs[aid], env_->cols, &vote_loc);
        if (maze_bottom_vote_group_from_loc(vote_loc, env_->cols) != vote_group) continue;
        if (direction > 0) right_count++;
        else if (direction < 0) left_count++;
    }

    maze_bottom_east_priority_phase_start_ = phase_start;
    maze_bottom_east_locked_right_vote_count_ = right_count;
    maze_bottom_east_locked_left_vote_count_ = left_count;

    const int old_direction = maze_bottom_east_preferred_direction_;
    int proposed_direction = maze_bottom_east_preferred_direction_;
    if (proposed_direction == 0) {
        if (right_count > left_count) proposed_direction = 1;
        else if (left_count > right_count) proposed_direction = -1;
    } else if (proposed_direction > 0) {
        if (left_count > right_count) proposed_direction = -1;
    } else {
        if (right_count > left_count) proposed_direction = 1;
    }

    bool group_corridor_has_reverse_agent = false;
    if (old_direction != 0 && proposed_direction != old_direction) {
        for (int aid = 0; aid < env_->num_of_agents; aid++) {
            if (aid >= static_cast<int>(env_->start_states.size())) continue;
            const int loc = env_->start_states[aid].location;
            if (loc < 0) continue;
            const int row = loc / env_->cols;
            const int col = loc % env_->cols;
            const bool in_group_corridor =
                row == kMazeBottomDoorRow && col >= kMazeBottomRightCol &&
                col <= kMazeBottomThirdDoorCol + 1;
            if (!in_group_corridor || aid >= static_cast<int>(trajLNS_.trajs.size()))
                continue;
            const int direction = maze_bottom_path_crossing_direction(trajLNS_.trajs[aid], env_->cols);
            if (direction == -old_direction) {
                group_corridor_has_reverse_agent = true;
                break;
            }
        }
    }
    if (old_direction != 0 && proposed_direction != old_direction &&
        group_corridor_has_reverse_agent)
        proposed_direction = old_direction;
    maze_bottom_east_preferred_direction_ = proposed_direction;
    return maze_bottom_east_preferred_direction_;
}

bool MazeTidalPlanner::maze_bottom_clearance_has_same_direction_agent(
    int vote_group,
    int direction,
    int except_agent,
    int agent_direction) const
{
    if (!env_ || env_->cols <= 0 || direction == 0 || agent_direction == 0)
        return false;
    const int agent_count = std::min<int>(env_->num_of_agents, env_->start_states.size());
    for (int aid = 0; aid < agent_count; aid++) {
        if (aid == except_agent || maze_bottom_vote_group(aid) != vote_group ||
            maze_bottom_vote_direction(aid) != direction)
            continue;
        const int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        const int row = loc / env_->cols;
        const int col = loc % env_->cols;
        const bool blocks_westward_exit =
            agent_direction < 0 && row == kMazeBottomDoorRow &&
            col >= kMazeBottomLeftCol && col <= kMazeBottomThirdDoorCol + 1;
        const bool blocks_eastward_exit =
            agent_direction > 0 &&
            ((row == kMazeBottomDoorRow &&
              col >= kMazeBottomEastDoorCol &&
              col <= kMazeBottomThirdDoorCol) ||
             ((row >= 28 && row <= kMazeBottomDoorRow && col == kMazeBottomLeftCol) ||
              (row == kMazeBottomDoorRow &&
               col >= kMazeBottomLeftCol - 2 &&
               col < kMazeBottomLeftCol)) ||
             (row >= 28 && row <= kMazeBottomDoorRow &&
              col == kMazeBottomThirdDoorCol + 1));
        if (blocks_westward_exit || blocks_eastward_exit) return true;
    }
    return false;
}

void MazeTidalPlanner::apply_maze_bottom_vote_tidal_priority(
    std::vector<double>& pibt_priority) const
{
    if (!is_maze_map() || !env_) return;
    const int east_preferred_direction = maze_bottom_periodic_preferred_direction(2);
    maze_bottom_locked_agent_votes_.assign(env_->num_of_agents, 9);
    maze_bottom_locked_agent_vote_locs_.assign(env_->num_of_agents, -1);
    maze_bottom_locked_agent_vote_groups_.assign(env_->num_of_agents, 0);

    const int agent_count = std::min<int>(env_->num_of_agents, pibt_priority.size());
    for (int aid = 0; aid < agent_count; aid++) {
        if (aid >= static_cast<int>(env_->start_states.size())) continue;
        const int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        const int row = loc / env_->cols;
        const int col = loc % env_->cols;
        if (!maze_bottom_vote_buffer_contains(row, col)) continue;
        if (col == kMazeBottomRightCol &&
            row >= kMazeBottomVoteMinRow + 1 &&
            row <= kMazeBottomDoorRow - 1) {
            pibt_priority[aid] +=
                (kMazeBottomDoorRow - 1 - row) * kMazeBottomCol17FrontPriorityStep;
        }
        const int direction = maze_bottom_vote_direction(aid);
        const int vote_group = maze_bottom_vote_group(aid);
        if (direction == 0 || vote_group != 2) continue;
        maze_bottom_locked_agent_votes_[aid] = direction;
        maze_bottom_locked_agent_vote_locs_[aid] = loc;
        maze_bottom_locked_agent_vote_groups_[aid] = vote_group;
        const int preferred_direction = east_preferred_direction;
        if (preferred_direction == 0) continue;
        pibt_priority[aid] += (direction == preferred_direction)
                                  ? kMazeBottomTidalPriorityDelta
                                  : -kMazeBottomTidalPriorityDelta;
        if (direction == -preferred_direction &&
            row == kMazeBottomDoorRow &&
            ((preferred_direction > 0 && col == kMazeBottomEastDoorCol) ||
             (preferred_direction < 0 && col == kMazeBottomThirdDoorCol)))
            pibt_priority[aid] += kMazeBottomExitReversePriorityBoost;
    }

    std::vector<std::pair<int, int>> east_refuge_agents;
    for (int aid = 0; aid < agent_count; aid++) {
        if (aid >= static_cast<int>(env_->start_states.size())) continue;
        const int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        const int row = loc / env_->cols;
        const int col = loc % env_->cols;
        if (col == kMazeBottomEastRefugeCol &&
            row >= kMazeBottomEastRefugeMinRow &&
            row <= kMazeBottomEastRefugeMaxRow)
            east_refuge_agents.emplace_back(row, aid);
    }
    if (east_refuge_agents.size() <= 1) return;
    std::vector<double> east_refuge_priorities;
    east_refuge_priorities.reserve(east_refuge_agents.size());
    for (const auto& [row, aid] : east_refuge_agents)
        east_refuge_priorities.push_back(pibt_priority[aid]);
    std::sort(east_refuge_agents.begin(), east_refuge_agents.end(),
              [](const auto& lhs, const auto& rhs) { return lhs.first > rhs.first; });
    std::sort(east_refuge_priorities.begin(), east_refuge_priorities.end(), std::greater<double>());
    for (size_t i = 0; i < east_refuge_agents.size(); i++)
        pibt_priority[east_refuge_agents[i].second] = east_refuge_priorities[i];
}

bool MazeTidalPlanner::maze_bottom_priority_forbids_move(int agent_id,
                                                         int from_loc,
                                                         int to_loc) const
{
    if (!is_maze_map() || !env_ || env_->cols <= 0) return false;
    const int vote_group = maze_bottom_vote_group(agent_id);
    const int preferred_direction = maze_bottom_periodic_preferred_direction(vote_group);
    const int agent_direction = maze_bottom_vote_direction(agent_id);
    if (preferred_direction == 0 || agent_direction == 0 ||
        agent_direction == preferred_direction)
        return false;

    const int from_row = from_loc / env_->cols;
    const int from_col = from_loc % env_->cols;
    const int to_row = to_loc / env_->cols;
    const int to_col = to_loc % env_->cols;
    if (maze_bottom_parking_contains(from_row, from_col)) {
        const bool leaving_parking = !maze_bottom_parking_contains(to_row, to_col);
        if (leaving_parking && vote_group == 2 &&
            ((preferred_direction > 0 && agent_direction < 0) ||
             (preferred_direction < 0 && agent_direction > 0)) &&
            !maze_bottom_clearance_has_same_direction_agent(vote_group,
                                                            preferred_direction,
                                                            agent_id,
                                                            agent_direction))
            return false;
        return !maze_bottom_parking_contains(to_row, to_col);
    }
    if (!maze_bottom_wander_loop_contains(from_row, from_col)) return false;
    const int next_loop_loc = maze_bottom_wander_loop_next_loc(env_, from_loc);
    return next_loop_loc >= 0 && to_loc != next_loop_loc;
}

int MazeTidalPlanner::maze_bottom_east_refuge_retreat_pibt_adjustment(
    int agent_id,
    int higher_id,
    int from_loc,
    int to_loc) const
{
    if (!is_maze_map() || !env_ || env_->cols <= 0 || higher_id < 0 ||
        from_loc < 0 || to_loc < 0 ||
        from_loc >= env_->rows * env_->cols || to_loc >= env_->rows * env_->cols)
        return 0;
    const int from_row = from_loc / env_->cols;
    const int from_col = from_loc % env_->cols;
    const int to_row = to_loc / env_->cols;
    const int to_col = to_loc % env_->cols;
    if (from_row != 31 || from_col != 22 || to_row != 30 || to_col != 22)
        return 0;
    const int vote_group = maze_bottom_vote_group(agent_id);
    if (vote_group != 2 || maze_bottom_vote_group(higher_id) != vote_group)
        return 0;
    const int preferred_direction = maze_bottom_periodic_preferred_direction(vote_group);
    const int agent_direction = maze_bottom_vote_direction(agent_id);
    const int higher_direction = maze_bottom_vote_direction(higher_id);
    if (preferred_direction == 0 || agent_direction == 0 || higher_direction == 0 ||
        agent_direction == preferred_direction || higher_direction != preferred_direction)
        return 0;
    return -1000;
}

bool MazeTidalPlanner::causal_pibt_with_extra_weights(
    int curr_id,
    int higher_id,
    std::vector<State>& prev_states,
    std::vector<State>& next_states,
    std::vector<int>& prev_decision,
    std::vector<int>& decision,
    std::vector<bool>& occupied,
    MazePlan::TrajLNS& lns)
{
    assert(next_states[curr_id].location == -1);
    const int prev_loc = prev_states[curr_id].location;
    const int prev_orientation = prev_states[curr_id].orientation;
    const int next[4] = {prev_loc + 1, prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
    const int orient_next = next[prev_orientation];
    assert(prev_loc >= 0 && prev_loc < static_cast<int>(lns.env->map.size()));
    if (higher_id == -1) occupied[prev_loc] = true;

    std::vector<int> neighbors;
    std::vector<MazePlan::PIBT_C> successors;
    MazePlan::getNeighborLocs(&(lns.neighbors), neighbors, prev_loc);
    for (int neighbor : neighbors) {
        assert(MazePlan::validateMove(prev_loc, neighbor, lns.env));
        if (maze_bottom_priority_forbids_move(curr_id, prev_loc, neighbor))
            continue;
        int heuristic = MazePlan::get_gp_h(lns, curr_id, neighbor);
        heuristic += get_extra_pibt_weight(extra_weights_, lns.env, prev_loc, neighbor);
        heuristic += maze_bottom_east_refuge_retreat_pibt_adjustment(curr_id, higher_id,
                                                                     prev_loc, neighbor);
        heuristic += get_full_guide_sequence_penalty(lns, curr_id, prev_loc, neighbor).penalty;
        successors.emplace_back(neighbor, heuristic, -1, rand());
    }

    int wait_heuristic = MazePlan::get_gp_h(lns, curr_id, prev_loc);
    wait_heuristic += get_full_guide_sequence_penalty(lns, curr_id, prev_loc, prev_loc).penalty;
    successors.emplace_back(prev_loc, wait_heuristic, -1, rand());

    std::sort(successors.begin(), successors.end(),
        [&](MazePlan::PIBT_C& a, MazePlan::PIBT_C& b) {
            if (a.heuristic == b.heuristic) {
                if (a.location == orient_next && b.location != orient_next) return true;
                if (a.location != orient_next && b.location == orient_next) return false;
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });

    for (auto& candidate : successors) {
        if (occupied[candidate.location] && !(higher_id == -1 && prev_loc == candidate.location))
            continue;
        assert(MazePlan::validateMove(prev_loc, candidate.location, lns.env));
        if (candidate.location == -1) continue;
        if (decision[candidate.location] != -1) continue;
        if (higher_id != -1 && prev_decision[candidate.location] == higher_id) continue;
        next_states[curr_id] = State(candidate.location, -1, -1);
        decision[candidate.location] = curr_id;
        if (prev_decision[candidate.location] != -1 &&
            next_states[prev_decision[candidate.location]].location == -1) {
            const int lower_id = prev_decision[candidate.location];
            if (!causal_pibt_with_extra_weights(lower_id, curr_id, prev_states, next_states,
                                                prev_decision, decision, occupied, lns)) {
                next_states[curr_id] = State(-1, -1, -1);
                decision[candidate.location] = -1;
                continue;
            }
        }
        return true;
    }
    next_states[curr_id] = State(prev_loc, -1, -1);
    decision[prev_loc] = curr_id;
    return false;
}

} // namespace optsvr
