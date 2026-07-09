// optsvr/planner/RoomTidalPlanner/RoomTidalPlanner.cpp
// Room Tidal Gate-Control Planner implementation
// Ported from LORR26_ssfc: room_c_5_6_tfo_periodic_vote_refuge_tidal_priority
#include "RoomTidalPlanner.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <queue>
#include <unordered_set>

// RoomPlanner infrastructure already included via RoomTidalPlanner.h
#include "RuntimeConfig.h"

namespace orz_engine {

// ═══════════════════════════════════════════════════════════════════
// Anonymous namespace: geometry helpers
// ═══════════════════════════════════════════════════════════════════
namespace {

constexpr double kIdleAgentPriority = -1e6;

constexpr int kRoom610HalfCrossTopMinRow = 29;
constexpr int kRoom610WaitingTopMaxRow = 31;
constexpr int kRoom610WaitingBottomMinRow = 33;
constexpr int kRoom610HalfCrossBottomMaxRow = 35;
constexpr int kRoom610WaitingPibtDirectionPenalty = 80;
constexpr int kRoom610WaitingPibtSouthTurnPenalty = 80;
constexpr int kRoom01TopHighwayMinRow = 8;
constexpr int kRoom01TopHighwayMaxRow = 10;
constexpr int kRoom01BottomHighwayMinRow = 11;
constexpr int kRoom01BottomHighwayMaxRow = 13;
constexpr int kRoom01HighwayMinCol = 13;
constexpr int kRoom01HighwayMaxCol = 19;

bool room_tidal_idle_current_low_priority_enabled() {
    return orz_engine::cfg().opt_room_idle_mode == 1;
}

int room_tidal_idle_target(const SharedEnvironment* env, int agent,
                           const std::vector<int>& dummy_goals) {
    if (!env || agent < 0 || agent >= static_cast<int>(env->start_states.size()))
        return -1;
    if (room_tidal_idle_current_low_priority_enabled() ||
        agent >= static_cast<int>(dummy_goals.size())) {
        return env->start_states[agent].location;
    }
    return dummy_goals[agent];
}

double room_tidal_idle_priority(int agent, const std::vector<double>& initial_priorities) {
    if (room_tidal_idle_current_low_priority_enabled())
        return kIdleAgentPriority;
    if (agent >= 0 && agent < static_cast<int>(initial_priorities.size()))
        return initial_priorities[agent];
    return 0.0;
}

// --- Room C geometry ---
bool room_c_window_contains(int row, int col) {
    return row >= 16 - 8 && row <= 16 + 8 && col >= 0 && col <= 1 + 8;
}
bool room_c_bottom_vote_buffer_contains(int row, int col) {
    return row >= 17 && row <= 24 && col >= 0 && col <= 9;
}
bool room_c_top_vote_buffer_contains(int row, int col) {
    return row >= 8 && row <= 15 && col >= 0 && col <= 9;
}
bool room_c_bottom_refuge_vote_buffer_contains(int row, int col) {
    return row >= 17 && row <= 26 && col >= 0 && col <= 9;
}
bool room_c_top_refuge_vote_buffer_contains(int row, int col) {
    return row >= 6 && row <= 15 && col >= 0 && col <= 9;
}
bool room_c_vote_buffer_contains_expanded(int row, int col) {
    return room_c_top_refuge_vote_buffer_contains(row, col) || room_c_bottom_refuge_vote_buffer_contains(row, col);
}
bool room_c_door_contains(int row, int col) {
    return row == 16 && col == 1;
}
bool room_c_refuge_island_contains(int row, int col) {
    return (row >= 6 && row <= 11 && col >= 1 && col <= 9) ||
           (row >= 17 && row <= 26 && col >= 5 && col <= 9);
}
bool room_c_top_refuge_island_contains(int row, int col) {
    return row < 16 && room_c_refuge_island_contains(row, col);
}
bool room_c_top_vote_non_refuge_contains(int row, int col) {
    return room_c_top_refuge_vote_buffer_contains(row, col) && !room_c_refuge_island_contains(row, col);
}
bool room_c_bottom_refuge_island_contains(int row, int col) {
    return row > 16 && room_c_refuge_island_contains(row, col);
}
bool room_c_bottom_vote_non_refuge_contains(int row, int col) {
    return room_c_bottom_refuge_vote_buffer_contains(row, col) && !room_c_refuge_island_contains(row, col);
}
bool room_c_bottom_vote_lower_half_contains(int row, int col) {
    return room_c_bottom_refuge_vote_buffer_contains(row, col) && row >= 22 && !room_c_refuge_island_contains(row, col);
}
bool room_c_top_vote_right_half_contains(int row, int col) {
    return room_c_top_refuge_vote_buffer_contains(row, col) && col >= 5 && !room_c_refuge_island_contains(row, col);
}
bool room_c_refuge_entry_penalty_exempt_move(int row, int col, int next_row, int next_col) {
    const bool bottom_left_entry =
        row >= 22 && row <= 26 &&
        next_row == row &&
        col == 4 && next_col == 5;
    const bool top_bottom_entry =
        col >= 5 && col <= 9 &&
        next_col == col &&
        row == 12 && next_row == 11;
    return bottom_left_entry || top_bottom_entry;
}
bool room_c_refuge_entry_move(int row, int col, int next_row, int next_col) {
    if (room_c_refuge_entry_penalty_exempt_move(row, col, next_row, next_col))
        return false;
    return (room_c_top_vote_non_refuge_contains(row, col) && room_c_top_refuge_island_contains(next_row, next_col)) ||
           (room_c_bottom_vote_non_refuge_contains(row, col) && room_c_bottom_refuge_island_contains(next_row, next_col));
}
int room_c_refuge_occupied_count(const SharedEnvironment* env) {
    if (!env || env->cols <= 0) return 0;
    int count = 0;
    for (const State& s : env->curr_states) {
        if (s.location >= 0 && s.location < (int)env->map.size() &&
            room_c_refuge_island_contains(s.location / env->cols, s.location % env->cols))
            count++;
    }
    return count;
}
bool room_c_refuge_has_vacancy(const SharedEnvironment* env) {
    if (!env || env->cols <= 0) return false;
    int cap = 0;
    for (int loc = 0; loc < (int)env->map.size(); loc++)
        if (env->map[loc] == 0 && room_c_refuge_island_contains(loc / env->cols, loc % env->cols)) cap++;
    return room_c_refuge_occupied_count(env) < cap;
}
int room_c_nearest_refuge_goal(const SharedEnvironment* env, int from_loc) {
    if (!env || env->cols <= 0 || from_loc < 0) return -1;
    const int ms = (int)env->map.size();
    std::vector<char> occ(ms, 0);
    for (const State& s : env->curr_states)
        if (s.location >= 0 && s.location < ms) occ[s.location] = 1;
    const int fr = from_loc / env->cols, fc = from_loc % env->cols;
    int bf = -1, bfs = INT_MAX, ba = -1, bas = INT_MAX;
    for (int loc = 0; loc < ms; loc++) {
        if (env->map[loc] != 0) continue;
        int r = loc / env->cols, c = loc % env->cols;
        if (!room_c_refuge_island_contains(r, c)) continue;
        int sc = std::abs(r - fr) + std::abs(c - fc);
        if (sc < bas) { bas = sc; ba = loc; }
        if (!occ[loc] && sc < bfs) { bfs = sc; bf = loc; }
    }
    return bf >= 0 ? bf : ba;
}
bool room_c_reverse_buffer_contains(int pref_dir, int row, int col) {
    if (room_c_door_contains(row, col)) return pref_dir != 0;
    if (room_c_refuge_island_contains(row, col)) return false;
    if (pref_dir > 0) return room_c_bottom_refuge_vote_buffer_contains(row, col);
    if (pref_dir < 0) return room_c_top_refuge_vote_buffer_contains(row, col);
    return false;
}
int room_c_clearance_score(int row, int col) {
    return std::abs(row - 16) + std::abs(col - 1);
}
int room_c_path_crossing_direction(const std::vector<int>& path, int cols) {
    if (cols <= 0 || path.size() < 2) return 0;
    for (size_t i = 1; i < path.size(); i++) {
        int p = path[i-1], c = path[i];
        if (p < 0 || c < 0) continue;
        int pr = p/cols, pc = p%cols, cr = c/cols, cc = c%cols;
        if (pc != 1 || cc != 1) continue;
        bool crosses = (pr == 16 && std::abs(cr - 16) == 1) || (cr == 16 && std::abs(pr - 16) == 1);
        if (!crosses) continue;
        return cr > pr ? 1 : -1;
    }
    return 0;
}

// --- Room 6-10 geometry ---
int room_6_10_path_crossing_direction(const std::vector<int>& path, int cols) {
    if (cols <= 0 || path.size() < 2) return 0;
    for (size_t i = 1; i < path.size(); i++) {
        int p = path[i-1], c = path[i];
        if (p < 0 || c < 0) continue;
        int pr = p/cols, pc = p%cols, cr = c/cols, cc = c%cols;
        if (pc != cc || (pc != 44 && pc != 47) || std::abs(cr - pr) != 1) continue;
        bool crosses = (pr == 32 && (cr == 31 || cr == 33)) || (cr == 32 && (pr == 31 || pr == 33));
        if (!crosses) continue;
        return cr > pr ? 1 : -1;
    }
    return 0;
}
bool room_6_10_waiting_left_half_contains(int row, int col) {
    bool in_rows = (row >= 24 && row <= 31) || (row >= 33 && row <= 40);
    return in_rows && col >= 42 && col <= 44;
}
bool room_6_10_waiting_right_half_contains(int row, int col) {
    bool in_rows = (row >= 24 && row <= 31) || (row >= 33 && row <= 40);
    return in_rows && col >= 45 && col <= 47;
}
bool room_6_10_waiting_half_cross_move(int row, int col, int nr, int nc) {
    auto in_cross_rows = [](int r) {
        return (r >= kRoom610HalfCrossTopMinRow &&
                r <= kRoom610WaitingTopMaxRow) ||
               (r >= kRoom610WaitingBottomMinRow &&
                r <= kRoom610HalfCrossBottomMaxRow);
    };
    return in_cross_rows(row) && in_cross_rows(nr) &&
           ((room_6_10_waiting_right_half_contains(row, col) && room_6_10_waiting_left_half_contains(nr, nc)) ||
            (room_6_10_waiting_left_half_contains(row, col) && room_6_10_waiting_right_half_contains(nr, nc)));
}
bool room_6_10_direction_context_contains(int row, int col) {
    return row >= 24 && row <= 40 && col >= 42 && col <= 48;
}
int room_6_10_waiting_pibt_direction_penalty(int row, int col, int nr, int nc) {
    (void)nc;
    if (room_6_10_waiting_right_half_contains(row, col))
        return nr > row ? kRoom610WaitingPibtDirectionPenalty : 0;
    if (room_6_10_waiting_left_half_contains(row, col))
        return nr < row ? kRoom610WaitingPibtDirectionPenalty : 0;
    return 0;
}
int room_6_10_waiting_pibt_south_turn_penalty(int row, int col, int prev_loc,
                                              int candidate_loc, int prev_orientation, int cols) {
    if (!room_6_10_waiting_right_half_contains(row, col)) return 0;
    if (prev_loc == candidate_loc) return 0;

    const int diff = candidate_loc - prev_loc;
    int target_orientation = prev_orientation;
    if (diff == 1) target_orientation = 0;
    else if (diff == cols) target_orientation = 1;
    else if (diff == -1) target_orientation = 2;
    else if (diff == -cols) target_orientation = 3;

    if (target_orientation == prev_orientation) return 0;
    const int first_turn_orientation =
        target_orientation == (prev_orientation + 3) % 4
            ? (prev_orientation + 3) % 4
            : (prev_orientation + 1) % 4;
    return first_turn_orientation == 1 ? kRoom610WaitingPibtSouthTurnPenalty : 0;
}

bool room_0_1_highway_top_half_contains(int row, int col) {
    return row >= kRoom01TopHighwayMinRow &&
           row <= kRoom01TopHighwayMaxRow &&
           col >= kRoom01HighwayMinCol &&
           col <= kRoom01HighwayMaxCol;
}
bool room_0_1_highway_bottom_half_contains(int row, int col) {
    return row >= kRoom01BottomHighwayMinRow &&
           row <= kRoom01BottomHighwayMaxRow &&
           col >= kRoom01HighwayMinCol &&
           col <= kRoom01HighwayMaxCol;
}
bool room_0_1_highway_half_cross_move(int row, int col, int nr, int nc) {
    return (room_0_1_highway_top_half_contains(row, col) &&
            room_0_1_highway_bottom_half_contains(nr, nc)) ||
           (room_0_1_highway_bottom_half_contains(row, col) &&
            room_0_1_highway_top_half_contains(nr, nc));
}

// --- Room 1-2 geometry ---
int room_1_2_path_crossing_direction(const std::vector<int>& path, int cols) {
    if (cols <= 0) return 0;
    for (size_t i = 1; i < path.size(); i++) {
        int p = path[i-1], c = path[i];
        if (p < 0 || c < 0) continue;
        int pr = p/cols, pc = p%cols, cr = c/cols, cc = c%cols;
        if (pr != cr || (pr != 11 && pr != 14) || std::abs(cc - pc) != 1) continue;
        bool crosses = (pc == 32 && (cc == 31 || cc == 33)) || (cc == 32 && (pc == 31 || pc == 33));
        if (!crosses) continue;
        return cc > pc ? 1 : -1;
    }
    return 0;
}
bool room_1_2_waiting_top_half_contains(int row, int col) {
    return row >= 7 && row <= 11 && col >= 24 && col <= 35;
}
bool room_1_2_waiting_bottom_half_contains(int row, int col) {
    return row >= 12 && row <= 15 && col >= 24 && col <= 35;
}
bool room_1_2_waiting_half_cross_move(int row, int col, int nr, int nc) {
    return (room_1_2_waiting_top_half_contains(row, col) && room_1_2_waiting_bottom_half_contains(nr, nc)) ||
           (room_1_2_waiting_bottom_half_contains(row, col) && room_1_2_waiting_top_half_contains(nr, nc));
}
bool room_1_2_direction_context_contains(int row, int col) {
    return row >= 7 && row <= 15 && col >= 24 && col <= 35;
}

// --- Room A geometry ---
bool room_a_right_vote_buffer_contains(int row, int col) {
    return row >= 17 && row <= 28 && col >= 27 && col <= 31;
}
bool room_a_left_vote_buffer_contains(int row, int col) {
    return row >= 17 && row <= 28 && col >= 33 && col <= 36;
}
bool room_a_vote_buffer_contains(int row, int col) {
    return room_a_left_vote_buffer_contains(row, col) || room_a_right_vote_buffer_contains(row, col);
}
bool room_a_refuge_island_contains(int row, int col) {
    return (row >= 22 && row <= 28 && col >= 27 && col <= 31) ||
           (row >= 22 && row <= 28 && col >= 33 && col <= 36);
}
bool room_a_left_refuge_island_contains(int row, int col) {
    return col < 32 && room_a_refuge_island_contains(row, col);
}
bool room_a_right_refuge_island_contains(int row, int col) {
    return col > 32 && room_a_refuge_island_contains(row, col);
}
bool room_a_left_vote_non_refuge_contains(int row, int col) {
    return room_a_left_vote_buffer_contains(row, col) && !room_a_refuge_island_contains(row, col);
}
bool room_a_right_vote_non_refuge_contains(int row, int col) {
    return room_a_right_vote_buffer_contains(row, col) && !room_a_refuge_island_contains(row, col);
}
bool room_a_left_refuge_entry_penalty_source(int row, int col) {
    return row >= 17 && row <= 21 && col == 35;
}
bool room_a_right_refuge_entry_penalty_source(int row, int col) {
    return row >= 17 && row <= 21 && col == 27;
}
bool room_a_refuge_entry_move(int row, int col, int nr, int nc) {
    return (room_a_left_refuge_entry_penalty_source(row, col) && room_a_right_refuge_island_contains(nr, nc)) ||
           (room_a_right_refuge_entry_penalty_source(row, col) && room_a_left_refuge_island_contains(nr, nc));
}
int room_a_refuge_occupied_count(const SharedEnvironment* env) {
    if (!env || env->cols <= 0) return 0;
    int count = 0;
    for (const State& s : env->curr_states)
        if (s.location >= 0 && s.location < (int)env->map.size() &&
            room_a_refuge_island_contains(s.location / env->cols, s.location % env->cols))
            count++;
    return count;
}
bool room_a_refuge_has_vacancy(const SharedEnvironment* env) {
    if (!env || env->cols <= 0) return false;
    int cap = 0;
    for (int loc = 0; loc < (int)env->map.size(); loc++)
        if (env->map[loc] == 0 && room_a_refuge_island_contains(loc / env->cols, loc % env->cols)) cap++;
    return room_a_refuge_occupied_count(env) < cap;
}
int room_a_nearest_refuge_goal(const SharedEnvironment* env, int from_loc) {
    if (!env || env->cols <= 0 || from_loc < 0) return -1;
    const int ms = (int)env->map.size();
    std::vector<char> occ(ms, 0);
    for (const State& s : env->curr_states)
        if (s.location >= 0 && s.location < ms) occ[s.location] = 1;
    int fr = from_loc / env->cols, fc = from_loc % env->cols;
    int bf = -1, bfs = INT_MAX, ba = -1, bas = INT_MAX;
    for (int loc = 0; loc < ms; loc++) {
        if (env->map[loc] != 0) continue;
        int r = loc / env->cols, c = loc % env->cols;
        if (!room_a_refuge_island_contains(r, c)) continue;
        int sc = std::abs(r - fr) + std::abs(c - fc);
        if (sc < bas) { bas = sc; ba = loc; }
        if (!occ[loc] && sc < bfs) { bfs = sc; bf = loc; }
    }
    return bf >= 0 ? bf : ba;
}
int room_a_path_crossing_direction(const std::vector<int>& path, int cols) {
    if (cols <= 0 || path.size() < 2) return 0;
    for (size_t i = 1; i < path.size(); i++) {
        int p = path[i-1], c = path[i];
        if (p < 0 || c < 0) continue;
        int pr = p/cols, pc = p%cols, cr = c/cols, cc = c%cols;
        if (pr != 20 || cr != 20) continue;
        bool crosses = (pc == 32 && std::abs(cc - 32) == 1) || (cc == 32 && std::abs(pc - 32) == 1);
        if (!crosses) continue;
        return cc > pc ? 1 : -1;
    }
    return 0;
}

// --- Room region / shortest path ---
int room_region_id_from_row_col(int row, int col) {
    int rr = std::min(3, row / 16), cc = std::min(3, col / 16);
    if (rr < 0 || cc < 0) return -1;
    return rr * 4 + cc;
}
const std::vector<std::pair<int,int>>& get_room_graph_edges() {
    static const std::vector<std::pair<int,int>> edges = {
        {0,1},{1,2},{2,3},{5,6},{6,7},{9,10},{10,11},{12,13},{14,15},
        {0,4},{2,6},{3,7},{4,8},{6,10},{8,12},{10,14},{11,15},
    };
    return edges;
}
bool room_shortest_path_contains_room(int from, int to, int target) {
    if (from < 0 || from >= 16 || to < 0 || to >= 16 || target < 0 || target >= 16) return false;
    if (from == target || to == target) return true;
    if (from == to) return false;
    std::array<std::vector<int>, 16> adj;
    for (auto& [a,b] : get_room_graph_edges()) { adj[a].push_back(b); adj[b].push_back(a); }
    std::array<int, 16> parent; parent.fill(-1);
    std::queue<int> q; q.push(from); parent[from] = from;
    while (!q.empty() && parent[to] == -1) {
        int r = q.front(); q.pop();
        for (int n : adj[r]) { if (parent[n] == -1) { parent[n] = r; q.push(n); } }
    }
    if (parent[to] == -1) return false;
    for (int r = to; r != from; r = parent[r]) if (r == target) return true;
    return false;
}
bool room_10_11_top_door_contains(int row, int col) {
    return row == 33 && col >= 47 && col <= 49;
}

// --- Guide sequence penalty ---
struct GuideSeqPenalty { int penalty = 0; int current_idx = -1; int candidate_idx = -1; int expected_idx = -1; bool on_path = false; bool next_step = false; };
int guide_closest_index(const std::vector<int>& path, int loc, int cols, bool* exact = nullptr) {
    if (exact) *exact = false;
    if (path.empty() || loc < 0 || cols <= 0) return -1;
    for (int i = 0; i < (int)path.size(); i++) if (path[i] == loc) { if (exact) *exact = true; return i; }
    int r = loc/cols, c = loc%cols, bi = 0, bd = INT_MAX;
    for (int i = 0; i < (int)path.size(); i++) {
        if (path[i] < 0) continue;
        int d = std::abs(path[i]/cols - r) + std::abs(path[i]%cols - c);
        if (d < bd) { bd = d; bi = i; }
    }
    return bi;
}
GuideSeqPenalty get_guide_penalty(const RoomPlan::TrajLNS& lns, int aid, int prev, int cand) {
    GuideSeqPenalty res;
    if (aid < 0 || aid >= (int)lns.trajs.size() || !lns.env || lns.env->cols <= 0) return res;
    auto& path = lns.trajs[aid];
    if (path.size() < 2) return res;
    auto& c = orz_engine::cfg();
    res.current_idx = guide_closest_index(path, prev, lns.env->cols);
    bool exact = false;
    res.candidate_idx = guide_closest_index(path, cand, lns.env->cols, &exact);
    res.on_path = exact;
    if (res.current_idx < 0 || res.candidate_idx < 0) return res;
    res.expected_idx = std::min(res.current_idx + 1, (int)path.size() - 1);
    res.next_step = cand == path[res.expected_idx];
    if (res.next_step) return res;
    if (cand == prev) {
        res.penalty = res.current_idx < (int)path.size() - 1 ? c.tidal_wait_penalty : 0;
        return res;
    }
    if (res.candidate_idx > res.current_idx) {
        return res;
    }
    res.penalty = c.tidal_backtrack_penalty + c.tidal_backtrack_step * (res.current_idx - res.candidate_idx);
    return res;
}

int get_extra_weight(const std::vector<std::vector<int>>& ew, const SharedEnvironment* env, int from, int to) {
    if (from < 0 || to < 0 || from >= (int)ew.size()) return 0;
    int d = RoomPlan::get_d(to - from, env);
    if (d < 0 || d >= (int)ew[from].size()) return 0;
    return ew[from][d];
}

int continued_errand_planning_goal(const SharedEnvironment* env, int agent_id,
                                   std::vector<int>* virtual_errand_indices = nullptr,
                                   std::vector<int>* virtual_errand_task_ids = nullptr) {
    if (!env || agent_id < 0 || agent_id >= (int)env->goal_locations.size() ||
        env->goal_locations[agent_id].empty())
        return -1;

    if (agent_id < (int)env->curr_task_schedule.size()) {
        const int task_id = env->curr_task_schedule[agent_id];
        const auto it = env->task_pool.find(task_id);
        if (it != env->task_pool.end() && agent_id < (int)env->start_states.size()) {
            const Task& task = it->second;
            const int next_index = task.idx_next_loc;
            if (next_index >= 0 && next_index < (int)task.locations.size()) {
                int planning_index = next_index;
                if (virtual_errand_indices) {
                    if (agent_id >= (int)virtual_errand_indices->size())
                        virtual_errand_indices->resize(agent_id + 1, -1);
                    int& virtual_index = (*virtual_errand_indices)[agent_id];
                    if (virtual_errand_task_ids) {
                        if (agent_id >= (int)virtual_errand_task_ids->size())
                            virtual_errand_task_ids->resize(agent_id + 1, -1);
                        int& virtual_task_id = (*virtual_errand_task_ids)[agent_id];
                        if (virtual_task_id != task_id) {
                            virtual_task_id = task_id;
                            virtual_index = next_index;
                        }
                    }
                    virtual_index = next_index;
                    planning_index = virtual_index;
                }
                return task.locations[planning_index];
            }
        }
    }

    for (const auto& task_entry : env->task_pool) {
        const Task& task = task_entry.second;
        const int next_index = task.idx_next_loc;
        if (task.agent_assigned == agent_id &&
            next_index >= 0 &&
            next_index < (int)task.locations.size()) {
            if (virtual_errand_indices) {
                if (agent_id >= (int)virtual_errand_indices->size())
                    virtual_errand_indices->resize(agent_id + 1, -1);
                if (virtual_errand_task_ids) {
                    if (agent_id >= (int)virtual_errand_task_ids->size())
                        virtual_errand_task_ids->resize(agent_id + 1, -1);
                    (*virtual_errand_task_ids)[agent_id] = task_entry.first;
                }
                (*virtual_errand_indices)[agent_id] = next_index;
            }
            return task.locations[next_index];
        }
    }

    const auto& goals = env->goal_locations[agent_id];
    int planning_index = 0;
    if (virtual_errand_indices) {
        if (agent_id >= (int)virtual_errand_indices->size())
            virtual_errand_indices->resize(agent_id + 1, -1);
        int& virtual_index = (*virtual_errand_indices)[agent_id];
        if (virtual_index < 0 || virtual_index >= (int)goals.size())
            virtual_index = 0;
        virtual_index = 0;
        planning_index = virtual_index;
    }
    return goals[planning_index].first;
}

int target_orientation_for_move(int from, int to, int cols) {
    const int diff = to - from;
    if (diff == 1) return 0;
    if (diff == cols) return 1;
    if (diff == -1) return 2;
    if (diff == -cols) return 3;
    return -1;
}

} // anonymous namespace

// ═══════════════════════════════════════════════════════════════════
// Public interface
// ═══════════════════════════════════════════════════════════════════
void RoomTidalPlanner::initialize(int preprocess_time_limit, SharedEnvironment* env)
{
    env_ = env;
    assert(env_->num_of_agents != 0);

    priorities_.resize(env_->num_of_agents);
    decision_.resize(env_->map.size(), -1);
    prev_states_.resize(env_->num_of_agents);
    next_states_.resize(env_->num_of_agents);
    decided_.resize(env_->num_of_agents, RoomPlan::DCR({-1, RoomPlan::DONE::DONE}));
    occupied_.resize(env_->map.size(), false);
    checked_.resize(env_->num_of_agents, false);
    ids_.resize(env_->num_of_agents);
    require_guide_path_.resize(env_->num_of_agents, false);
    virtual_errand_indices_.assign(env_->num_of_agents, -1);
    virtual_errand_task_ids_.assign(env_->num_of_agents, -1);
    virtual_errand_reference_locs_.assign(env_->num_of_agents, -1);
    room_6_10_last_direction_.assign(env_->num_of_agents, 0);
    room_1_2_last_direction_.assign(env_->num_of_agents, 0);
    for (int i = 0; i < (int)ids_.size(); i++) ids_[i] = i;

    initialize_extra_weights();
    if (is_room_map()) build_room_fixed_dual_lane_highway();

    mt1_.seed(0);
    srand(0);

    // Initialize RoomPlan global heuristics/neighbors (required by TrajLNS)
    RoomPlan::init_heuristics(env_);

    new (&trajLNS_) RoomPlan::TrajLNS(env_, RoomPlan::global_heuristictable, RoomPlan::global_neighbors);
    trajLNS_.init_mem();

    std::shuffle(ids_.begin(), ids_.end(), mt1_);
    for (int i = 0; i < (int)ids_.size(); i++)
        priorities_[ids_[i]] = ((double)(ids_.size() - i)) / ((double)(ids_.size() + 1));
    initial_priorities_ = priorities_;
    initialized_ = true;
}

void RoomTidalPlanner::plan(int time_limit, std::vector<std::vector<Action>>& actions,
                            SharedEnvironment* env, int num_steps)
{
    env_ = env;
    actions.clear();
    if (!env_ || env_->num_of_agents <= 0) return;
    actions.resize(env_->num_of_agents);
    if (num_steps <= 0) return;

    const auto episode_start = std::chrono::steady_clock::now();
    const TimePoint episode_deadline = episode_start + std::chrono::milliseconds(std::max(0, time_limit));
    int pibt_time = RoomPlan::PIBT_RUNTIME_PER_100_AGENTS * env_->num_of_agents / 100;
    if (pibt_time <= 0) pibt_time = 1;
    const int flow_budget_ms = std::max(0, time_limit - std::max(RoomPlan::MIN_PIBT_TIME, pibt_time * num_steps) - RoomPlan::TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE);
    TimePoint flow_end_time = episode_start + std::chrono::milliseconds(flow_budget_ms);

    std::vector<double> local_priority = priorities_;
    const std::vector<State> original_states = env_->start_states;
    const int original_curr_timestep = env_->curr_timestep;
    const int original_system_timestep = env_->system_timestep;
    const int original_timestep = env_->system_timestep >= 0 ? env_->system_timestep : env_->curr_timestep;
    const std::vector<RoomPlan::DCR> original_decided = decided_;

    decided_.assign(env_->num_of_agents, RoomPlan::DCR({-1, RoomPlan::DONE::DONE}));
    initialize_dummy_goals_if_needed();
    setup_multistep_episode_state(flow_end_time, local_priority);
    update_guide_paths_once(flow_end_time);
    priorities_ = local_priority;

    for (int step = 0; step < num_steps; step++)
    {
        if (std::chrono::steady_clock::now() >= episode_deadline) break;
        std::vector<Action> one_step_actions;
        if (step > 0) refresh_multistep_step_state(local_priority);
        env_->system_timestep = original_timestep + step;
        if (step >= 3)
            run_tidal_no_collision_once(local_priority, one_step_actions);
        else
            run_tidal_pibt_once(local_priority, one_step_actions);
        append_actions_and_rollout_states(actions, one_step_actions);
    }

    env_->start_states = original_states;
    env_->curr_timestep = original_curr_timestep;
    env_->system_timestep = original_system_timestep;
    decided_ = original_decided;
}

// ═══════════════════════════════════════════════════════════════════
// Internal methods
// ═══════════════════════════════════════════════════════════════════
bool RoomTidalPlanner::is_room_map() const {
    return env_ && env_->rows == 64 && env_->cols == 64;
}

void RoomTidalPlanner::initialize_extra_weights() {
    if (!env_) return;
    extra_weights_.assign(env_->rows * env_->cols, std::vector<int>(4, 0));
}

void RoomTidalPlanner::build_room_fixed_dual_lane_highway()
{
    if (!env_ || !is_room_map()) return;
    const int cols = env_->cols, ms = env_->rows * env_->cols;
    auto add_pen = [&](int r, int c, int dir) {
        if (r < 0 || r >= env_->rows || c < 0 || c >= env_->cols || dir < 0 || dir >= 4) return;
        int loc = r * cols + c;
        if (loc < 0 || loc >= ms || env_->map[loc] != 0) return;
        extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], BOTTLENECK_HIGHWAY_PENALTY);
    };
    auto add_pen_v = [&](int r, int c, int dir, int v) {
        if (r < 0 || r >= env_->rows || c < 0 || c >= env_->cols || dir < 0 || dir >= 4) return;
        int loc = r * cols + c;
        if (loc < 0 || loc >= ms || env_->map[loc] != 0) return;
        extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], v);
    };

    // Room 6-10 dual lane
    add_pen(ROOM_6_10_DUAL_LANE_WALL_ROW, ROOM_6_10_DOWN_LANE_COL, 3);
    add_pen(ROOM_6_10_DUAL_LANE_BOTTOM_ROW, ROOM_6_10_DOWN_LANE_COL, 3);
    add_pen(ROOM_6_10_DUAL_LANE_TOP_ROW, ROOM_6_10_UP_LANE_COL, 1);
    add_pen(ROOM_6_10_DUAL_LANE_WALL_ROW, ROOM_6_10_UP_LANE_COL, 1);
    add_pen_v(ROOM_10_11_DUAL_LANE_TOP_ROW, ROOM_10_11_DUAL_LANE_LEFT_COL, 0,
              ROOM_10_11_TOP_DOOR_AVOID_PENALTY);
    add_pen_v(ROOM_10_11_DUAL_LANE_TOP_ROW, ROOM_10_11_DUAL_LANE_WALL_COL, 0,
              ROOM_10_11_TOP_DOOR_AVOID_PENALTY);
    add_pen_v(ROOM_6_7_DUAL_LANE_LOWER_ROW, ROOM_6_7_DUAL_LANE_RIGHT_COL, 2,
              ROOM_6_7_LOWER_DOOR_LEFTWARD_PENALTY);
    add_pen_v(ROOM_6_7_DUAL_LANE_LOWER_ROW, ROOM_6_7_DUAL_LANE_WALL_COL, 2,
              ROOM_6_7_LOWER_DOOR_LEFTWARD_PENALTY);
    add_pen(ROOM_1_2_DUAL_LANE_TOP_ROW, ROOM_1_2_DUAL_LANE_LEFT_COL, 0);
    add_pen(ROOM_1_2_DUAL_LANE_TOP_ROW, ROOM_1_2_DUAL_LANE_WALL_COL, 0);
    add_pen(ROOM_1_2_DUAL_LANE_BOTTOM_ROW, ROOM_1_2_DUAL_LANE_RIGHT_COL, 2);
    add_pen(ROOM_1_2_DUAL_LANE_BOTTOM_ROW, ROOM_1_2_DUAL_LANE_WALL_COL, 2);

    for (int r = ROOM_0_1_TOP_HIGHWAY_MIN_ROW; r <= ROOM_0_1_TOP_HIGHWAY_MAX_ROW; r++)
        for (int c = ROOM_0_1_HIGHWAY_MIN_COL; c <= ROOM_0_1_HIGHWAY_MAX_COL; c++)
            add_pen(r, c, 0);
    for (int r = ROOM_0_1_BOTTOM_HIGHWAY_MIN_ROW; r <= ROOM_0_1_BOTTOM_HIGHWAY_MAX_ROW; r++)
        for (int c = ROOM_0_1_HIGHWAY_MIN_COL; c <= ROOM_0_1_HIGHWAY_MAX_COL; c++)
            add_pen(r, c, 2);

    for (int r = ROOM_1_5_TOP_CONNECTOR_MIN_ROW; r <= ROOM_1_5_TOP_CONNECTOR_MAX_ROW; r++) {
        for (int c = ROOM_1_5_TOP_CONNECTOR_DOWN_MIN_COL; c <= ROOM_1_5_TOP_CONNECTOR_DOWN_MAX_COL; c++)
            add_pen(r, c, 3);
        for (int c = ROOM_1_5_TOP_CONNECTOR_UP_MIN_COL; c <= ROOM_1_5_TOP_CONNECTOR_UP_MAX_COL; c++)
            add_pen(r, c, 1);
    }
    for (int r = ROOM_1_5_BOTTOM_CONNECTOR_MIN_ROW; r <= ROOM_1_5_BOTTOM_CONNECTOR_MAX_ROW; r++) {
        for (int c = ROOM_1_5_BOTTOM_CONNECTOR_DOWN_MIN_COL; c <= ROOM_1_5_BOTTOM_CONNECTOR_DOWN_MAX_COL; c++)
            add_pen(r, c, 3);
        for (int c = ROOM_1_5_BOTTOM_CONNECTOR_UP_MIN_COL; c <= ROOM_1_5_BOTTOM_CONNECTOR_UP_MAX_COL; c++)
            add_pen(r, c, 1);
    }

    for (int r = ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MIN_ROW; r <= ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MAX_ROW; r++)
        for (int c = ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MIN_COL; c <= ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_MAX_COL; c++)
            add_pen_v(r, c, 2, ROOM_6_10_TOP_RIGHT_WAIT_LEFT_CUT_PENALTY);

    const int delta[4] = {1, cols, -1, -cols};
    for (int loc = 0; loc < ms; loc++) {
        if (env_->map[loc] != 0) continue;
        int row = loc / cols, col = loc % cols;
        for (int dir = 0; dir < 4; dir++) {
            int next = loc + delta[dir];
            if (next < 0 || next >= ms || env_->map[next] != 0 || !RoomPlan::validateMove(loc, next, env_)) continue;
            int nr = next / cols, nc = next % cols;
            if (room_6_10_waiting_left_half_contains(row, col) && nr < row)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_6_10_waiting_right_half_contains(row, col) && nr > row)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_6_10_waiting_half_cross_move(row, col, nr, nc))
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_HALF_CROSS_BASE_PENALTY);
            if (room_1_2_waiting_top_half_contains(row, col) && nc > col)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_1_2_waiting_bottom_half_contains(row, col) && nc < col)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_1_2_waiting_half_cross_move(row, col, nr, nc))
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_HALF_CROSS_BASE_PENALTY);
            if (room_0_1_highway_half_cross_move(row, col, nr, nc))
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_HALF_CROSS_BASE_PENALTY);
            if (room_c_bottom_vote_lower_half_contains(row, col) && nr < row)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_c_top_vote_right_half_contains(row, col) && nc < col)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_a_left_refuge_entry_penalty_source(row, col) && nc < col)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_a_right_refuge_entry_penalty_source(row, col) && nc > col)
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_WAITING_SIDE_BASE_PENALTY);
            if (room_c_refuge_entry_move(row, col, nr, nc))
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_C_REFUGE_ENTRY_PENALTY);
            if (room_a_refuge_entry_move(row, col, nr, nc))
                extra_weights_[loc][dir] = std::max(extra_weights_[loc][dir], ROOM_A_REFUGE_ENTRY_PENALTY);
        }
    }
}

bool RoomTidalPlanner::update_room_guide_traj(int agent_id, const TimePoint* deadline)
{
    if (agent_id < 0 || agent_id >= (int)trajLNS_.trajs.size()) return false;
    // Use RoomPlan's weighted A* search
    if (!RoomPlan::update_traj(trajLNS_, agent_id, deadline, &extra_weights_)) return false;

    // Room C refuge rerouting
    if (agent_id < (int)env_->start_states.size()) {
        int pref_c = room_c_periodic_preferred_direction();
        int dir_c = room_c_down_priority_direction(agent_id);
        int loc = env_->start_states[agent_id].location;
        if (loc >= 0 && loc < (int)env_->map.size()) {
            int r = loc / env_->cols, c = loc % env_->cols;
            if (pref_c != 0 && dir_c != 0 && dir_c != pref_c &&
                room_c_vote_buffer_contains_expanded(r, c) && !room_c_refuge_island_contains(r, c) &&
                room_c_refuge_has_vacancy(env_)) {
                int refuge = room_c_nearest_refuge_goal(env_, loc);
                if (refuge >= 0 && refuge != loc) {
                    auto base_traj = trajLNS_.trajs[agent_id];
                    auto base_gn = trajLNS_.goal_nodes[agent_id];
                    int base_task = trajLNS_.tasks[agent_id];
                    RoomPlan::remove_traj(trajLNS_, agent_id);
                    trajLNS_.tasks[agent_id] = refuge;
                    if (RoomPlan::update_traj(trajLNS_, agent_id, deadline, &extra_weights_)) return true;
                    trajLNS_.tasks[agent_id] = base_task;
                    trajLNS_.trajs[agent_id] = std::move(base_traj);
                    trajLNS_.goal_nodes[agent_id] = base_gn;
                    RoomPlan::add_traj(trajLNS_, agent_id);
                    RoomPlan::update_dist_2_path(trajLNS_, agent_id);
                    return true;
                }
            }
        }
    }

    // Room A refuge rerouting
    if (agent_id < (int)env_->start_states.size()) {
        int pref_a = room_a_periodic_preferred_direction();
        int dir_a = room_a_right_priority_direction(agent_id);
        int loc = env_->start_states[agent_id].location;
        if (loc >= 0 && loc < (int)env_->map.size()) {
            int r = loc / env_->cols, c = loc % env_->cols;
            if (pref_a != 0 && dir_a != 0 && dir_a != pref_a &&
                room_a_vote_buffer_contains(r, c) && !room_a_refuge_island_contains(r, c) &&
                room_a_refuge_has_vacancy(env_)) {
                int refuge = room_a_nearest_refuge_goal(env_, loc);
                if (refuge >= 0 && refuge != loc) {
                    auto base_traj = trajLNS_.trajs[agent_id];
                    auto base_gn = trajLNS_.goal_nodes[agent_id];
                    int base_task = trajLNS_.tasks[agent_id];
                    RoomPlan::remove_traj(trajLNS_, agent_id);
                    trajLNS_.tasks[agent_id] = refuge;
                    if (RoomPlan::update_traj(trajLNS_, agent_id, deadline, &extra_weights_)) return true;
                    trajLNS_.tasks[agent_id] = base_task;
                    trajLNS_.trajs[agent_id] = std::move(base_traj);
                    trajLNS_.goal_nodes[agent_id] = base_gn;
                    RoomPlan::add_traj(trajLNS_, agent_id);
                    RoomPlan::update_dist_2_path(trajLNS_, agent_id);
                    return true;
                }
            }
        }
    }

    // Room 6-10 / 1-2 direction-aware re-routing
    int cross_dir = room_6_10_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols);
    int r12_dir = room_1_2_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols);
    if (cross_dir != 0 && agent_id < (int)room_6_10_last_direction_.size())
        room_6_10_last_direction_[agent_id] = cross_dir;
    else if (cross_dir == 0 && agent_id < (int)room_6_10_last_direction_.size() && !trajLNS_.trajs[agent_id].empty()) {
        int fl = trajLNS_.trajs[agent_id].front();
        if (room_6_10_direction_context_contains(fl / env_->cols, fl % env_->cols))
            cross_dir = room_6_10_last_direction_[agent_id];
        else room_6_10_last_direction_[agent_id] = 0;
    }
    if (r12_dir != 0 && agent_id < (int)room_1_2_last_direction_.size())
        room_1_2_last_direction_[agent_id] = r12_dir;
    else if (r12_dir == 0 && agent_id < (int)room_1_2_last_direction_.size() && !trajLNS_.trajs[agent_id].empty()) {
        int fl = trajLNS_.trajs[agent_id].front();
        if (room_1_2_direction_context_contains(fl / env_->cols, fl % env_->cols))
            r12_dir = room_1_2_last_direction_[agent_id];
        else room_1_2_last_direction_[agent_id] = 0;
    }

    int curr_room = room_region_id_from_row_col(env_->start_states[agent_id].location / env_->cols, env_->start_states[agent_id].location % env_->cols);
    int tr = trajLNS_.tasks[agent_id] >= 0 ? trajLNS_.tasks[agent_id] / env_->cols : -1;
    int tc = trajLNS_.tasks[agent_id] >= 0 ? trajLNS_.tasks[agent_id] % env_->cols : -1;
    int task_room = trajLNS_.tasks[agent_id] >= 0 ? room_region_id_from_row_col(tr, tc) : -1;
    bool goes_6 = room_shortest_path_contains_room(curr_room, task_room, 6);
    if (cross_dir == 0 && goes_6) return true;

    auto base_traj = trajLNS_.trajs[agent_id];
    auto base_gn = trajLNS_.goal_nodes[agent_id];
    RoomPlan::remove_traj(trajLNS_, agent_id);
    if (RoomPlan::update_traj(trajLNS_, agent_id, deadline, &extra_weights_)) return true;
    trajLNS_.trajs[agent_id] = std::move(base_traj);
    trajLNS_.goal_nodes[agent_id] = base_gn;
    RoomPlan::add_traj(trajLNS_, agent_id);
    RoomPlan::update_dist_2_path(trajLNS_, agent_id);
    return true;
}

void RoomTidalPlanner::frank_wolfe_room_guide(TimePoint timelimit)
{
    RoomPlan::update_fw_metrics(trajLNS_);
    auto replan_order = trajLNS_.fw_metrics;
    std::sort(replan_order.begin(), replan_order.end(),
        [](RoomPlan::FW_Metric& a, RoomPlan::FW_Metric& b) {
            if (a.deviation != b.deviation) return a.deviation > b.deviation;
            if (a.last_replan_t != b.last_replan_t) return a.last_replan_t < b.last_replan_t;
            return a.rand > b.rand;
        });
    int count = 0;
    while (std::chrono::steady_clock::now() < timelimit) {
        int idx = count % env_->num_of_agents;
        int aid = replan_order[idx].id;
        count++;
        if (trajLNS_.traj_dists[aid].empty() || trajLNS_.trajs[aid].empty()) continue;
        auto old_traj = trajLNS_.trajs[aid];
        auto old_gn = trajLNS_.goal_nodes[aid];
        RoomPlan::remove_traj(trajLNS_, aid);
        if (!update_room_guide_traj(aid, &timelimit)) {
            trajLNS_.trajs[aid] = std::move(old_traj);
            trajLNS_.goal_nodes[aid] = old_gn;
            RoomPlan::add_traj(trajLNS_, aid);
            RoomPlan::update_dist_2_path(trajLNS_, aid);
            break;
        }
    }
}

State RoomTidalPlanner::rollout_next_state(const State& state, Action action, const SharedEnvironment* env)
{
    State next = state;
    next.timestep = state.timestep + 1;
    if (action == Action::CR) { next.orientation = (state.orientation + 1) % 4; return next; }
    if (action == Action::CCR) { next.orientation = (state.orientation + 3) % 4; return next; }
    if (action != Action::FW) return next;
    int delta = 0;
    if (state.orientation == 0) delta = 1;
    if (state.orientation == 1) delta = env->cols;
    if (state.orientation == 2) delta = -1;
    if (state.orientation == 3) delta = -env->cols;
    int nl = state.location + delta;
    if (nl >= 0 && nl < (int)env->map.size() && RoomPlan::validateMove(state.location, nl, env))
        next.location = nl;
    return next;
}

void RoomTidalPlanner::initialize_dummy_goals_if_needed()
{
    if (env_->curr_timestep != 0) return;
    dummy_goals_.resize(env_->num_of_agents);
    for (int i = 0; i < env_->num_of_agents; i++)
        dummy_goals_[i] = env_->start_states[i].location;
}

void RoomTidalPlanner::setup_multistep_episode_state(TimePoint flow_end_time, std::vector<double>& local_priority)
{
    if (virtual_errand_indices_.size() != (size_t)env_->num_of_agents)
        virtual_errand_indices_.assign(env_->num_of_agents, -1);
    if (virtual_errand_task_ids_.size() != (size_t)env_->num_of_agents)
        virtual_errand_task_ids_.assign(env_->num_of_agents, -1);
    if (virtual_errand_reference_locs_.size() != (size_t)env_->num_of_agents)
        virtual_errand_reference_locs_.assign(env_->num_of_agents, -1);
    for (int i = 0; i < env_->num_of_agents && i < (int)env_->start_states.size(); i++) {
        if (virtual_errand_reference_locs_[i] < 0)
            virtual_errand_reference_locs_[i] = env_->start_states[i].location;
    }
    prev_decision_.clear();
    prev_decision_.resize(env_->map.size(), -1);
    std::fill(decision_.begin(), decision_.end(), -1);
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() < flow_end_time) {
            for (auto& [goal_loc, _] : env_->goal_locations[i]) {
                if (trajLNS_.heuristics.find(goal_loc) == trajLNS_.heuristics.end() || trajLNS_.heuristics[goal_loc].empty())
                    RoomPlan::init_heuristic(trajLNS_.heuristics[goal_loc], env_, goal_loc);
            }
        }
        const bool has_goal = !env_->goal_locations[i].empty();
        if (!has_goal) {
            if (i < (int)virtual_errand_indices_.size()) virtual_errand_indices_[i] = -1;
            if (i < (int)virtual_errand_task_ids_.size()) virtual_errand_task_ids_[i] = -1;
            trajLNS_.tasks[i] = room_tidal_idle_target(env_, i, dummy_goals_);
            local_priority[i] = room_tidal_idle_priority(i, initial_priorities_);
        } else {
            trajLNS_.tasks[i] = continued_errand_planning_goal(
                env_, i, &virtual_errand_indices_, &virtual_errand_task_ids_);
        }
        require_guide_path_[i] = trajLNS_.trajs[i].empty() || trajLNS_.trajs[i].back() != trajLNS_.tasks[i];
        assert(env_->start_states[i].location >= 0);
        prev_states_[i] = env_->start_states[i];
        next_states_[i] = State();
        prev_decision_[env_->start_states[i].location] = i;
        if (decided_[i].loc == -1) { decided_[i].loc = env_->start_states[i].location; }
        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = RoomPlan::DONE::DONE;
        if (decided_[i].state == RoomPlan::DONE::NOT_DONE) { decision_[decided_[i].loc] = i; next_states_[i] = State(decided_[i].loc, -1, -1); }
        const bool reached_current_task =
            !env_->goal_locations[i].empty() &&
            env_->start_states[i].location == trajLNS_.tasks[i];
        if (!has_goal) local_priority[i] = room_tidal_idle_priority(i, initial_priorities_);
        else if (reached_current_task) local_priority[i] = 0.0;
        else if (require_guide_path_[i]) local_priority[i] = initial_priorities_[i];
        else local_priority[i] += 1;
        if (has_goal && trajLNS_.neighbors[env_->start_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void RoomTidalPlanner::update_guide_paths_once(TimePoint flow_end_time)
{
    for (int i = 0; i < env_->num_of_agents; i++) {
        if (std::chrono::steady_clock::now() > flow_end_time) break;
        if (require_guide_path_[i]) {
            auto old_traj = trajLNS_.trajs[i];
            if (!trajLNS_.trajs[i].empty()) RoomPlan::remove_traj(trajLNS_, i);
            if (!update_room_guide_traj(i, &flow_end_time)) {
                trajLNS_.trajs[i] = old_traj;
                if (!old_traj.empty()) RoomPlan::add_traj(trajLNS_, i);
                break;
            }
        }
    }
    frank_wolfe_room_guide(flow_end_time);
}

void RoomTidalPlanner::refresh_multistep_step_state(std::vector<double>& local_priority)
{
    virtual_errand_reference_locs_.assign(env_->num_of_agents, -1);
    for (int i = 0; i < env_->num_of_agents && i < (int)env_->start_states.size(); i++)
        virtual_errand_reference_locs_[i] = env_->start_states[i].location;
    prev_decision_.clear();
    prev_decision_.resize(env_->map.size(), -1);
    std::fill(decision_.begin(), decision_.end(), -1);
    for (int i = 0; i < env_->num_of_agents; i++) {
        const bool has_goal = !env_->goal_locations[i].empty();
        if (!has_goal) {
            if (i < (int)virtual_errand_indices_.size()) virtual_errand_indices_[i] = -1;
            if (i < (int)virtual_errand_task_ids_.size()) virtual_errand_task_ids_[i] = -1;
            trajLNS_.tasks[i] = room_tidal_idle_target(env_, i, dummy_goals_);
            local_priority[i] = room_tidal_idle_priority(i, initial_priorities_);
        } else {
            trajLNS_.tasks[i] = continued_errand_planning_goal(
                env_, i, &virtual_errand_indices_, &virtual_errand_task_ids_);
            if (env_->start_states[i].location == trajLNS_.tasks[i])
                local_priority[i] = 0.0;
            else
                local_priority[i] += 1;
        }
        assert(env_->start_states[i].location >= 0);
        prev_states_[i] = env_->start_states[i];
        next_states_[i] = State();
        prev_decision_[env_->start_states[i].location] = i;
        if (prev_states_[i].location == decided_[i].loc) decided_[i].state = RoomPlan::DONE::DONE;
        if (decided_[i].state == RoomPlan::DONE::NOT_DONE) { decision_[decided_[i].loc] = i; next_states_[i] = State(decided_[i].loc, -1, -1); }
        if (has_goal && trajLNS_.neighbors[env_->start_states[i].location].size() == 1)
            local_priority[i] += 10;
    }
}

void RoomTidalPlanner::run_tidal_pibt_once(std::vector<double>& local_priority, std::vector<Action>& one_step_actions)
{
    std::vector<double> pibt_priority = local_priority;
    const int pref_c = room_c_periodic_preferred_direction();
    const int pref_a = room_a_periodic_preferred_direction();

    if (pref_c != 0) {
        for (int aid = 0; aid < env_->num_of_agents; aid++) {
            int d = room_c_down_priority_direction(aid);
            if (d == pref_c) pibt_priority[aid] += ROOM_C_DOWN_PRIORITY_DELTA;
            else if (d == -pref_c) pibt_priority[aid] -= ROOM_C_DOWN_PRIORITY_DELTA;
        }
    }
    if (pref_a != 0) {
        for (int aid = 0; aid < env_->num_of_agents; aid++) {
            if (aid >= (int)env_->start_states.size()) continue;
            const int loc = env_->start_states[aid].location;
            if (loc < 0) continue;
            const int row = loc / env_->cols;
            const int col = loc % env_->cols;
            if (!room_a_vote_buffer_contains(row, col)) continue;
            int d = room_a_right_priority_direction(aid);
            if (d == pref_a) pibt_priority[aid] += ROOM_A_RIGHT_PRIORITY_DELTA;
            else if (d == -pref_a) pibt_priority[aid] -= ROOM_A_RIGHT_PRIORITY_DELTA;
        }
    }

    std::sort(ids_.begin(), ids_.end(), [&](int a, int b) { return pibt_priority[a] > pibt_priority[b]; });
    std::fill(occupied_.begin(), occupied_.end(), false);

    for (int i : ids_) {
        if (decided_[i].state == RoomPlan::DONE::NOT_DONE) continue;
        if (next_states_[i].location == -1) {
            assert(prev_states_[i].location >= 0 && prev_states_[i].location < (int)env_->map.size());
            causal_pibt_with_room_c_down_priority(i, -1, prev_states_, next_states_, prev_decision_, decision_, occupied_, trajLNS_);
        }
    }

    one_step_actions.resize(env_->num_of_agents);
    for (int id : ids_) {
        if (next_states_[id].location != -1) decision_[next_states_[id].location] = -1;
        if (next_states_[id].location >= 0) decided_[id] = RoomPlan::DCR({next_states_[id].location, RoomPlan::DONE::NOT_DONE});
        one_step_actions[id] = RoomPlan::getAction(prev_states_[id], decided_[id].loc, env_);
        checked_[id] = false;
    }
    for (int id = 0; id < env_->num_of_agents; id++) {
        if (!checked_[id] && one_step_actions[id] == Action::FW)
            RoomPlan::moveCheck(id, checked_, decided_, one_step_actions, prev_decision_);
    }
    prev_states_ = next_states_;
}

void RoomTidalPlanner::run_tidal_no_collision_once(std::vector<double>& local_priority,
                                                   std::vector<Action>& one_step_actions)
{
    (void)local_priority;
    auto score_action = [&](int aid, Action action) {
        const State before = prev_states_[aid];
        const State after = rollout_next_state(before, action, env_);
        if (after.location != before.location &&
            room_c_down_priority_forbids_move(aid, before.location, after.location))
            return std::numeric_limits<long long>::max() / 4;

        long long score =
            static_cast<long long>(RoomPlan::get_gp_h(trajLNS_, aid, after.location)) * 50;
        score += get_extra_weight(extra_weights_, env_, before.location, after.location);
        if (action == Action::FW && after.location == before.location)
            score += 1000000;
        if (aid >= 0 && aid < (int)trajLNS_.tasks.size() &&
            after.location == trajLNS_.tasks.at(aid))
            score -= 1200;
        if (action == Action::FW)
            score -= 8;
        else if (action == Action::CR || action == Action::CCR)
            score += 10;
        else
            score += 2;
        return score;
    };

    one_step_actions.assign(env_->num_of_agents, Action::W);
    const std::array<Action, 4> action_order = {Action::FW, Action::CR, Action::CCR, Action::W};
    for (int aid = 0; aid < env_->num_of_agents; aid++) {
        Action best_action = Action::W;
        long long best_score = std::numeric_limits<long long>::max();
        for (Action action : action_order) {
            const long long score = score_action(aid, action);
            if (score < best_score) {
                best_score = score;
                best_action = action;
            }
        }
        one_step_actions[aid] = best_action;
    }
}

bool RoomTidalPlanner::causal_pibt_with_room_c_down_priority(
    int curr_id, int higher_id,
    std::vector<State>& prev_st, std::vector<State>& next_st,
    std::vector<int>& prev_dec, std::vector<int>& dec,
    std::vector<bool>& occ, RoomPlan::TrajLNS& lns)
{
    assert(next_st[curr_id].location == -1);
    int prev_loc = prev_st[curr_id].location;
    int prev_orient = prev_st[curr_id].orientation;
    int next_arr[4] = {prev_loc + 1, prev_loc + lns.env->cols, prev_loc - 1, prev_loc - lns.env->cols};
    int orien_next = next_arr[prev_orient];
    assert(prev_loc >= 0 && prev_loc < (int)lns.env->map.size());
    if (higher_id == -1) occ[prev_loc] = true;

    std::vector<int> neighbors;
    std::vector<RoomPlan::PIBT_C> successors;
    RoomPlan::getNeighborLocs(&(lns.neighbors), neighbors, prev_loc);
    int prev_row = prev_loc / lns.env->cols, prev_col = prev_loc % lns.env->cols;
    int pref_c = room_c_periodic_preferred_direction();
    int curr_dir = room_c_down_priority_direction(curr_id);

    // Direction tracking
    int r610_dir = 0;
    if (curr_id >= 0 && curr_id < (int)lns.trajs.size()) {
        r610_dir = room_6_10_path_crossing_direction(lns.trajs[curr_id], lns.env->cols);
        if (r610_dir != 0 && curr_id < (int)room_6_10_last_direction_.size())
            room_6_10_last_direction_[curr_id] = r610_dir;
        else if (r610_dir == 0 && curr_id < (int)room_6_10_last_direction_.size()) {
            if (room_6_10_direction_context_contains(prev_row, prev_col))
                r610_dir = room_6_10_last_direction_[curr_id];
            else room_6_10_last_direction_[curr_id] = 0;
        }
    }
    int r12_dir = 0;
    if (curr_id >= 0 && curr_id < (int)lns.trajs.size()) {
        r12_dir = room_1_2_path_crossing_direction(lns.trajs[curr_id], lns.env->cols);
        if (r12_dir != 0 && curr_id < (int)room_1_2_last_direction_.size())
            room_1_2_last_direction_[curr_id] = r12_dir;
        else if (r12_dir == 0 && curr_id < (int)room_1_2_last_direction_.size()) {
            if (room_1_2_direction_context_contains(prev_row, prev_col))
                r12_dir = room_1_2_last_direction_[curr_id];
            else room_1_2_last_direction_[curr_id] = 0;
        }
    }

    // Eviction logic (non-refuge layout only, disabled for refuge)
    bool evict_c = false; // refuge layout: eviction handled by forbids_move
    int pref_a = room_a_periodic_preferred_direction();
    bool evict = evict_c;
    const bool green_flow_active = room_c_green_flow_applies(curr_id, prev_loc);
    bool green_flow_move_candidate_available = false;

    for (auto& nb : neighbors) {
        assert(RoomPlan::validateMove(prev_loc, nb, lns.env));
        if (room_c_down_priority_forbids_move(curr_id, prev_loc, nb) ||
            room_a_right_priority_forbids_move(curr_id, prev_loc, nb))
            continue;
        if (green_flow_active &&
            !room_c_green_flow_allows_candidate(curr_id, prev_loc, nb, prev_st.at(curr_id)))
            continue;
        if (green_flow_active) green_flow_move_candidate_available = true;
        int h = RoomPlan::get_gp_h(lns, curr_id, nb);
        h += get_extra_weight(extra_weights_, lns.env, prev_loc, nb);
        auto gp = get_guide_penalty(lns, curr_id, prev_loc, nb);
        h += gp.penalty;
        const int nb_row = nb / lns.env->cols;
        const int nb_col = nb % lns.env->cols;
        if (!gp.next_step) {
            h += room_6_10_waiting_pibt_direction_penalty(prev_row, prev_col, nb_row, nb_col);
            h += room_6_10_waiting_pibt_south_turn_penalty(prev_row, prev_col, prev_loc,
                                                           nb, prev_orient, lns.env->cols);
        }
        successors.emplace_back(nb, h, -1, rand());
    }

    if (!evict || successors.empty()) {
        if (!(green_flow_active && green_flow_move_candidate_available && !successors.empty())) {
            int wh = RoomPlan::get_gp_h(lns, curr_id, prev_loc);
            auto gp = get_guide_penalty(lns, curr_id, prev_loc, prev_loc);
            wh += gp.penalty;
            successors.emplace_back(prev_loc, wh, -1, rand());
        }
    }

    std::sort(successors.begin(), successors.end(),
        [&](RoomPlan::PIBT_C& a, RoomPlan::PIBT_C& b) {
            if (a.heuristic == b.heuristic) {
                if (a.location == orien_next && b.location != orien_next) return true;
                if (a.location != orien_next && b.location == orien_next) return false;
                return a.tie_breaker < b.tie_breaker;
            }
            return a.heuristic < b.heuristic;
        });

    for (auto& nx : successors) {
        if (occ[nx.location] && !(higher_id == -1 && prev_loc == nx.location)) continue;
        if (nx.location == -1 || dec[nx.location] != -1) continue;
        if (higher_id != -1 && prev_dec[nx.location] == higher_id) continue;
        next_st[curr_id] = State(nx.location, -1, -1);
        dec[nx.location] = curr_id;
        if (prev_dec[nx.location] != -1 && next_st[prev_dec[nx.location]].location == -1) {
            int lower = prev_dec[nx.location];
            if (!causal_pibt_with_room_c_down_priority(lower, curr_id, prev_st, next_st, prev_dec, dec, occ, lns))
                continue;
        }
        return true;
    }
    next_st[curr_id] = State(prev_loc, -1, -1);
    dec[prev_loc] = curr_id;
    return false;
}

void RoomTidalPlanner::append_actions_and_rollout_states(std::vector<std::vector<Action>>& actions, const std::vector<Action>& one_step_actions)
{
    for (int aid = 0; aid < env_->num_of_agents; aid++)
        actions[aid].push_back(one_step_actions[aid]);
    std::vector<State> rolled(env_->num_of_agents);
    for (int aid = 0; aid < env_->num_of_agents; aid++)
        rolled[aid] = rollout_next_state(env_->start_states[aid], one_step_actions[aid], env_);
    env_->start_states = rolled;
    env_->curr_timestep += 1;
}

// ═══════════════════════════════════════════════════════════════════
// Direction functions
// ═══════════════════════════════════════════════════════════════════
int RoomTidalPlanner::room_c_down_priority_direction(int agent_id) const {
    if (!is_room_map() || !env_ || agent_id < 0 || agent_id >= env_->num_of_agents) return 0;
    if (agent_id < (int)trajLNS_.trajs.size()) {
        int d = room_c_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols);
        if (d != 0) return d;
    }
    return 0;
}

bool RoomTidalPlanner::room_c_down_flow_active() const {
    if (!is_room_map()) return false;
    return room_c_periodic_preferred_direction() != 0;
}

int RoomTidalPlanner::room_c_periodic_preferred_direction() const {
    if (!is_room_map() || !env_) return 0;
    int ts = env_->system_timestep >= 0 ? env_->system_timestep : env_->curr_timestep;
    int period = orz_engine::cfg().tidal_period;
    int phase_start = (ts / period) * period;
    if (room_c_priority_phase_start_ == phase_start) return room_c_preferred_direction_;

    int down = 0, up = 0;
    room_c_locked_agent_votes_.assign(env_->num_of_agents, 9);
    room_c_locked_agent_vote_locs_.assign(env_->num_of_agents, -1);
    for (int aid = 0; aid < env_->num_of_agents; aid++) {
        if (aid >= (int)env_->start_states.size()) continue;
        int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        int r = loc / env_->cols, c = loc % env_->cols;
        if (!room_c_vote_buffer_contains_expanded(r, c)) continue;
        room_c_locked_agent_votes_[aid] = 0;
        room_c_locked_agent_vote_locs_[aid] = loc;
        int d = room_c_down_priority_direction(aid);
        room_c_locked_agent_votes_[aid] = d;
        if (d > 0) down++; else if (d < 0) up++;
    }
    room_c_priority_phase_start_ = phase_start;
    room_c_locked_down_vote_count_ = down;
    room_c_locked_up_vote_count_ = up;
    if (room_c_preferred_direction_ == 0) {
        if (down > up) room_c_preferred_direction_ = 1;
        else if (up > down) room_c_preferred_direction_ = -1;
    } else if (room_c_preferred_direction_ > 0) {
        if (up > 2 * down) room_c_preferred_direction_ = -1;
    } else {
        if (down > 2 * up) room_c_preferred_direction_ = 1;
    }
    return room_c_preferred_direction_;
}

bool RoomTidalPlanner::room_c_down_priority_forbids_move(int agent_id, int from_loc, int to_loc) const {
    if (!room_c_down_flow_active()) return false;
    int pref = room_c_periodic_preferred_direction();
    int dir = room_c_down_priority_direction(agent_id);
    int fr = from_loc / env_->cols, fc = from_loc % env_->cols;
    int tr = to_loc / env_->cols, tc = to_loc % env_->cols;
    // Refuge layout: forbid leaving refuge into non-refuge vote buffer
    if (pref != 0 && dir != 0 && dir != pref &&
        ((room_c_top_refuge_island_contains(fr, fc) && room_c_top_vote_non_refuge_contains(tr, tc)) ||
         (room_c_bottom_refuge_island_contains(fr, fc) && room_c_bottom_vote_non_refuge_contains(tr, tc))))
        return true;
    return false;
}

bool RoomTidalPlanner::room_c_green_flow_applies(int agent_id, int from_loc) const {
    if (!is_room_map() ||
        !env_ ||
        from_loc < 0 ||
        from_loc >= (int)env_->map.size())
        return false;

    const int preferred_direction = room_c_periodic_preferred_direction();
    if (preferred_direction == 0 || room_c_down_priority_direction(agent_id) != preferred_direction)
        return false;

    const int row = from_loc / env_->cols;
    const int col = from_loc % env_->cols;
    if (col < 0 || col > ROOM_C_VOTE_BUFFER_COL_MAX) return false;

    if (preferred_direction < 0) {
        return row >= ROOM_C_DOOR_ROW - 1 &&
               row <= ROOM_C_DOOR_ROW + ROOM_C_VOTE_BUFFER_ROW_RADIUS + ROOM_C_VOTE_BUFFER_OUTWARD_EXTRA;
    }
    return row >= ROOM_C_DOOR_ROW - ROOM_C_VOTE_BUFFER_ROW_RADIUS - ROOM_C_VOTE_BUFFER_OUTWARD_EXTRA &&
           row <= ROOM_C_DOOR_ROW + 1;
}

bool RoomTidalPlanner::room_c_green_flow_allows_candidate(int agent_id,
                                                          int from_loc,
                                                          int candidate_loc,
                                                          const State& prev_state) const {
    if (!room_c_green_flow_applies(agent_id, from_loc)) return true;
    if (!env_ ||
        candidate_loc < 0 ||
        candidate_loc >= (int)env_->map.size() ||
        candidate_loc == from_loc ||
        !RoomPlan::validateMove(from_loc, candidate_loc, env_))
        return false;

    const int target_orientation = target_orientation_for_move(from_loc, candidate_loc, env_->cols);
    if (target_orientation < 0) return false;

    const int preferred_direction = room_c_periodic_preferred_direction();
    const auto orientation_allowed = [&](int orientation) {
        if (preferred_direction < 0) return orientation == 2 || orientation == 3;
        return orientation == 1 || orientation == 2;
    };

    if (preferred_direction < 0) {
        const int row = from_loc / env_->cols;
        const int col = from_loc % env_->cols;
        const bool east_escape = (row == 18 || row == 20) && col == 0 && target_orientation == 0;
        if (east_escape) return true;
    }

    if (!orientation_allowed(target_orientation)) return false;
    if (prev_state.orientation == target_orientation) return true;

    const int clockwise_orientation = (prev_state.orientation + 1) % 4;
    const int counter_clockwise_orientation = (prev_state.orientation + 3) % 4;
    return orientation_allowed(clockwise_orientation) ||
           orientation_allowed(counter_clockwise_orientation);
}

Action RoomTidalPlanner::room_c_green_flow_action_for_candidate(int agent_id,
                                                                const State& prev_state,
                                                                int candidate_loc) const {
    if (!env_ || candidate_loc < 0 || candidate_loc >= (int)env_->map.size())
        return Action::W;
    if (!room_c_green_flow_applies(agent_id, prev_state.location) ||
        candidate_loc == prev_state.location) {
        State prev_copy = prev_state;
        return RoomPlan::getAction(prev_copy, candidate_loc, env_);
    }

    const int target_orientation = target_orientation_for_move(prev_state.location, candidate_loc, env_->cols);
    if (target_orientation < 0 || target_orientation == prev_state.orientation) {
        State prev_copy = prev_state;
        return RoomPlan::getAction(prev_copy, candidate_loc, env_);
    }

    const int preferred_direction = room_c_periodic_preferred_direction();
    const auto orientation_allowed = [&](int orientation) {
        if (preferred_direction < 0) {
            const int row = prev_state.location / env_->cols;
            const int col = prev_state.location % env_->cols;
            const bool east_escape = (row == 18 || row == 20) && col == 0 && target_orientation == 0;
            if (east_escape) return orientation == 0 || orientation == 2 || orientation == 3;
            return orientation == 2 || orientation == 3;
        }
        return orientation == 1 || orientation == 2;
    };

    const int clockwise_orientation = (prev_state.orientation + 1) % 4;
    const int counter_clockwise_orientation = (prev_state.orientation + 3) % 4;
    if (orientation_allowed(counter_clockwise_orientation) &&
        !orientation_allowed(clockwise_orientation))
        return Action::CCR;
    if (orientation_allowed(clockwise_orientation) &&
        !orientation_allowed(counter_clockwise_orientation))
        return Action::CR;
    if ((target_orientation - prev_state.orientation + 4) % 4 == 3)
        return Action::CCR;
    return Action::CR;
}

int RoomTidalPlanner::room_a_right_priority_direction(int agent_id) const {
    if (!is_room_map() || !env_ || agent_id < 0 || agent_id >= env_->num_of_agents) return 0;
    if (agent_id < (int)trajLNS_.trajs.size()) {
        int d = room_a_path_crossing_direction(trajLNS_.trajs[agent_id], env_->cols);
        if (d != 0) return d;
    }
    return 0;
}

bool RoomTidalPlanner::room_a_right_flow_active() const {
    if (!is_room_map()) return false;
    return room_a_periodic_preferred_direction() != 0;
}

int RoomTidalPlanner::room_a_periodic_preferred_direction() const {
    if (!is_room_map() || !env_) return 0;
    int ts = env_->system_timestep >= 0 ? env_->system_timestep : env_->curr_timestep;
    int period = orz_engine::cfg().tidal_period;
    int phase_start = (ts / period) * period;
    if (room_a_priority_phase_start_ == phase_start) return room_a_preferred_direction_;

    int right = 0, left = 0;
    room_a_locked_agent_votes_.assign(env_->num_of_agents, 9);
    room_a_locked_agent_vote_locs_.assign(env_->num_of_agents, -1);
    for (int aid = 0; aid < env_->num_of_agents; aid++) {
        if (aid >= (int)env_->start_states.size()) continue;
        int loc = env_->start_states[aid].location;
        if (loc < 0) continue;
        int r = loc / env_->cols, c = loc % env_->cols;
        if (!room_a_vote_buffer_contains(r, c)) continue;
        room_a_locked_agent_votes_[aid] = 0;
        room_a_locked_agent_vote_locs_[aid] = loc;
        int d = room_a_right_priority_direction(aid);
        room_a_locked_agent_votes_[aid] = d;
        if (d > 0) right++; else if (d < 0) left++;
    }
    room_a_priority_phase_start_ = phase_start;
    room_a_locked_right_vote_count_ = right;
    room_a_locked_left_vote_count_ = left;
    if (room_a_preferred_direction_ == 0) {
        if (right > left) room_a_preferred_direction_ = 1;
        else if (left > right) room_a_preferred_direction_ = -1;
    } else if (room_a_preferred_direction_ > 0) {
        if (left > 2 * right) room_a_preferred_direction_ = -1;
    } else {
        if (right > 2 * left) room_a_preferred_direction_ = 1;
    }
    return room_a_preferred_direction_;
}

bool RoomTidalPlanner::room_a_right_priority_forbids_move(int agent_id, int from_loc, int to_loc) const {
    if (!room_a_right_flow_active()) return false;
    int pref = room_a_periodic_preferred_direction();
    int dir = room_a_right_priority_direction(agent_id);
    if (pref == 0 || dir == 0 || dir == pref) return false;
    int fr = from_loc / env_->cols, fc = from_loc % env_->cols;
    int tr = to_loc / env_->cols, tc = to_loc % env_->cols;
    if ((room_a_left_refuge_island_contains(fr, fc) && room_a_right_vote_non_refuge_contains(tr, tc)) ||
        (room_a_right_refuge_island_contains(fr, fc) && room_a_left_vote_non_refuge_contains(tr, tc)))
        return true;
    return false;
}

} // namespace orz_engine
