// control_zone.h — v9-safe2 with max safety
#pragma once
#include "SharedEnv.h"
#include "centerline.h"
#include "narrow_passage.h"
#include "config.h"
#include "pibt_lock.h"
#include <vector>
#include <algorithm>
#include <climits>

namespace ControlZone {

inline void init(const SharedEnvironment* env) {
    printf("[CTRL] Edge pushout v9-safe2 init: %d agents\n", env->num_of_agents);
}

inline bool is_reverse(const SharedEnvironment* env, int aid, int a_main, int lock_dir) {
    if (lock_dir == 0) return false;
    auto rev = [&](int loc) -> bool {
        int lm, lb;
        Centerline::project(loc, lm, lb);
        return (lm >= 0 && ((lock_dir > 0 && a_main - lm >= 5) || (lock_dir < 0 && lm - a_main >= 5)));
    };
    int tid1 = env->curr_task_schedule[aid];
    if (tid1 >= 0) {
        auto it = env->task_pool.find(tid1);
        if (it != env->task_pool.end())
            for (int loc : it->second.locations) if (rev(loc)) return true;
    }
    int tid2 = (aid < PIBT_Lock::MAX_AGENTS) ? PIBT_Lock::g_parked_task[aid] : -1;
    if (tid2 >= 0) {
        auto it = env->task_pool.find(tid2);
        if (it != env->task_pool.end())
            for (int loc : it->second.locations) if (rev(loc)) return true;
    }
    if (tid1 < 0 && tid2 < 0) return true;
    return false;
}

inline int safe_loc_in_square(int target_si, int cols, const SharedEnvironment* env, int fallback_loc) {
    if (target_si < 0 || target_si >= (int)Centerline::g_main_squares.size()) return fallback_loc;
    const auto& sq = Centerline::g_main_squares[target_si];
    int tl = sq.cy * cols + sq.cx;
    if (tl >= 0 && tl < (int)env->map.size() && env->map[tl] == 0
        && (int)Centerline::g_cell_square.size() > tl && Centerline::g_cell_square[tl] == target_si)
        return tl;
    // Search nearby walkable cells
    for (int dr = -3; dr <= 3; dr++) {
        for (int dc = -3; dc <= 3; dc++) {
            int nr = sq.cy + dr, nc = sq.cx + dc;
            if (nr < 0 || nr >= env->rows || nc < 0 || nc >= env->cols) continue;
            int nl = nr * cols + nc;
            if (nl >= 0 && nl < (int)env->map.size() && env->map[nl] == 0
                && (int)Centerline::g_cell_square.size() > nl
                && Centerline::g_cell_square[nl] == target_si)
                return nl;
        }
    }
    return fallback_loc;
}

inline int find_slot(const SharedEnvironment* env, int a_zid, int n24c) {
    int best = -1, best_d2 = INT_MAX;
    for (int dzx = -1; dzx <= 1; dzx++) {
        for (int dzy = -1; dzy <= 1; dzy++) {
            int tz = a_zid + dzy * n24c + dzx;
            if (tz < 0 || tz >= n24c * Centerline::g_n24r) continue;
            for (size_t si = 0; si < NarrowPassage::g_edge_slots.size(); si++) {
                if (NarrowPassage::g_slot_assign[si] >= 0) continue;
                const auto& s = NarrowPassage::g_edge_slots[si];
                int sz = s.zy * n24c + s.zx;
                if (sz != tz) continue;
                int d2 = dzx*dzx + dzy*dzy;
                if (d2 < best_d2) { best_d2 = d2; best = (int)si; }
            }
        }
    }
    return best;
}

inline void update(SharedEnvironment* env) {
    int n24c = Centerline::g_n24c;
    int cols = env->cols;
    std::fill(NarrowPassage::g_slot_assign.begin(), NarrowPassage::g_slot_assign.end(), -1);
    
    for (int a = 0; a < env->num_of_agents; a++) {
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) continue;
        if (a >= PIBT_Lock::MAX_AGENTS) continue;

        int si = (a_loc < (int)Centerline::g_cell_square.size()) 
                 ? Centerline::g_cell_square[a_loc] : -1;
        if (si < 0 || si >= (int)Centerline::g_main_squares.size()) {
            PIBT_Lock::g_cold_goals[a] = -1;
            PIBT_Lock::g_agent_bound_lock[a] = -1;
            continue;
        }
        
        const auto& sq = Centerline::g_main_squares[si];
        int lock_dir = sq.lock_dir;
        
        int a_zid = (a_loc / cols / 24) * n24c + ((a_loc % cols) / 24);
        NarrowPassage::CorridorClass cls = (a_zid < (int)NarrowPassage::g_zone_cls.size())
                                           ? NarrowPassage::g_zone_cls[a_zid] 
                                           : NarrowPassage::VERY_WIDE;
        int density = (a_zid < (int)Centerline::g_zone24_density.size())
                      ? Centerline::g_zone24_density[a_zid] : 0;
        
        bool is_narrow = (cls == NarrowPassage::SUPER_NARROW || cls == NarrowPassage::NARROW);
        
        // N/SN fallback lock
        if (lock_dir == 0 && is_narrow && density >= 5) {
            int hint_plus = 0, hint_minus = 0;
            for (int si2 = si - 3; si2 <= si + 3; si2++) {
                if (si2 < 0 || si2 >= (int)Centerline::g_main_squares.size()) continue;
                if (si2 == (int)si) continue;
                int d = Centerline::g_main_squares[si2].lock_dir;
                if (d > 0) hint_plus++;
                else if (d < 0) hint_minus++;
            }
            lock_dir = (hint_plus >= hint_minus) ? +1 : -1;
        }
        
        if (lock_dir == 0) {
            PIBT_Lock::g_cold_goals[a] = -1;
            PIBT_Lock::g_agent_bound_lock[a] = -1;
            continue;
        }
        
        bool is_controlled = false;
        if (is_narrow) is_controlled = true;
        else if (cls == NarrowPassage::MODERATE) is_controlled = (density >= CONTROL_M_THRESHOLD);
        else if (cls == NarrowPassage::WIDE) is_controlled = (density >= CONTROL_W_THRESHOLD);
        
        int am, ab;
        Centerline::project(a_loc, am, ab);
        if (am < 0) continue;
        
        // Check existing cold_goal
        int cg = PIBT_Lock::g_cold_goals[a];
        if (cg >= 0) {
            if (a_loc == cg) {
                if (!is_controlled) {
                    PIBT_Lock::g_cold_goals[a] = -1;
                    PIBT_Lock::g_agent_bound_lock[a] = -1;
                }
            } else if (!is_controlled) {
                PIBT_Lock::g_cold_goals[a] = -1;
                PIBT_Lock::g_agent_bound_lock[a] = -1;
            }
            continue;
        }
        
        if (!is_controlled) continue;
        if (!is_reverse(env, a, am, lock_dir)) continue;
        
        int new_goal = -1;
        if (is_narrow) {
            int target_si = si + lock_dir * 2;
            if (target_si >= 0 && target_si < (int)Centerline::g_main_squares.size())
                new_goal = safe_loc_in_square(target_si, cols, env, a_loc);
        } else {
            int slot_id = find_slot(env, a_zid, n24c);
            if (slot_id >= 0) {
                NarrowPassage::g_slot_assign[slot_id] = a;
                new_goal = NarrowPassage::g_edge_slots[slot_id].loc;
            } else {
                int target_si = si + lock_dir * 1;
                if (target_si >= 0 && target_si < (int)Centerline::g_main_squares.size())
                    new_goal = safe_loc_in_square(target_si, cols, env, a_loc);
            }
        }
        
        if (new_goal >= 0 && new_goal != a_loc) {
            PIBT_Lock::g_cold_goals[a] = new_goal;
            PIBT_Lock::g_agent_bound_lock[a] = 1;
        }
    }
}

} // namespace ControlZone
