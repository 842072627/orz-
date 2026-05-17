// control_zone.h — Reverse-agent priority pushout (v6)
// No edge slots. When zone is direction-locked and hot:
// - Agents going AGAINST lock_dir get PIBT priority to exit the zone
// - Agents going WITH lock_dir continue normally
// - No wasted walking to far-away edge slots
#pragma once
#include "SharedEnv.h"
#include "centerline.h"
#include "narrow_passage.h"
#include "config.h"
#include "pibt_lock.h"
#include <vector>
#include <algorithm>
#include <cstring>

namespace ControlZone {

enum ControlLevel {
    LV0_FREE   = 0,
    LV1_SOFT   = 1,
    LV2_HARD   = 2,
};

static std::vector<ControlLevel> g_zone_level;
static std::vector<int> g_zone_control_dir;
static std::vector<bool> g_agent_in_control;      // [aid] → 是否在管控区中需要push
static std::vector<int> g_agent_push_target;       // [aid] → PIBT push目标坐标, -1=无

static bool g_lock_reached_dead_end = false;

inline void init(const SharedEnvironment* env) {
    int nz = Centerline::g_n24c * Centerline::g_n24r;
    g_zone_level.assign(nz, LV0_FREE);
    g_zone_control_dir.assign(nz, 0);
    g_agent_in_control.assign(env->num_of_agents, false);
    g_agent_push_target.assign(env->num_of_agents, -1);
    printf("[CTRL] Reverse-pushout v6 init: %d zones\n", nz);
}

inline bool is_zone_controlled(int zid) {
    return zid >= 0 && zid < (int)g_zone_level.size() && g_zone_level[zid] >= LV1_SOFT;
}

// 判断agent在管控zone中是否"走反"：task目标在lock_dir反方向
inline bool is_going_reverse(const SharedEnvironment* env, int aid, int a_loc, int control_dir, int zid) {
    if (control_dir == 0) return false;
    int cur = env->curr_task_schedule[aid];
    if (cur < 0) return true; // 无task → 视为反向
    if (!env->goal_locations[aid].empty()) {
        int next_goal = env->goal_locations[aid].front().first;
        int tm, tb;
        Centerline::project(next_goal, tm, tb);
        if (tm >= 0) {
            int am, ab;
            Centerline::project(a_loc, am, ab);
            if (am >= 0) {
                int delta = tm - am;
                // delta的符号与control_dir相反 → reverse
                if ((control_dir > 0 && delta < -5) || (control_dir < 0 && delta > 5))
                    return true;
            }
        }
    }
    return false;
}

// 找lock_dir方向上的下一个出口zone（非锁定的zone）
inline int find_exit_zone(int zid, int control_dir) {
    if (control_dir == 0) return -1;
    int n24c = Centerline::g_n24c;
    int n24r = Centerline::g_n24r;
    int zr = zid / n24c, zc = zid % n24c;
    
    // 沿lock_dir走
    for (int step = 1; step <= 3; step++) {
        int nr = zr + (control_dir * step); // 简化：lock_dir≈±1，对应纵向前后
        if (nr < 0 || nr >= n24r) return zid; // 地图边界也是出口
        int nzid = nr * n24c + zc;
        if (!is_zone_controlled(nzid)) return nzid; // 到非管控区了
    }
    return -1;
}

// 计算agent在管控区内的"挤推目标"：沿lock_dir方向出zone的边界位置
inline int calc_push_target(const SharedEnvironment* env, int a_loc, int zid, int control_dir) {
    int n24c = Centerline::g_n24c;
    int zr = zid / n24c, zc = zid % n24c;
    int target_zid = find_exit_zone(zid, control_dir);
    if (target_zid < 0) return -1;
    
    // 目标：出zone的边界中点
    int tzr = target_zid / n24c, tzc = target_zid % n24c;
    int tx = tzc * 24 + 12;
    int ty = tzr * 24 + 12;
    
    // 找中心线上最接近该zone边界点的位置作为目标
    int best_loc = -1, best_dist = INT_MAX;
    for (size_t ci = 0; ci < Centerline::g_centerline_cells.size(); ci++) {
        int loc = Centerline::g_centerline_cells[ci];
        int lr = loc / env->cols, lc = loc % env->cols;
        int dist = (lr-ty)*(lr-ty) + (lc-tx)*(lc-tx);
        if (dist < best_dist) {
            best_dist = dist;
            best_loc = loc;
        }
    }
    return best_loc;
}

// ====== 每tick更新管控级别（每tick） ======
inline void update_control_levels(const SharedEnvironment* env) {
    int n24c = Centerline::g_n24c;
    int n24r = Centerline::g_n24r;
    int nz = n24c * n24r;
    
    g_lock_reached_dead_end = false;
    
    if ((int)g_zone_level.size() != nz) {
        g_zone_level.assign(nz, LV0_FREE);
        g_zone_control_dir.assign(nz, 0);
    }
    
    std::fill(g_zone_level.begin(), g_zone_level.end(), LV0_FREE);
    std::fill(g_zone_control_dir.begin(), g_zone_control_dir.end(), 0);
    
    for (int z = 0; z < nz; z++) {
        bool has_cl = (NarrowPassage::g_zone_centerline_cells[z] > 0);
        if (!has_cl) continue;
        
        int zr = z / n24c, zc = z % n24c;
        int zcx = zc * 24 + 12, zcy = zr * 24 + 12;
        int lock_dir = 0;
        int best_d2 = INT_MAX;
        for (size_t si = 0; si < Centerline::g_main_squares.size(); si++) {
            int dx = zcx - Centerline::g_main_squares[si].cx;
            int dy = zcy - Centerline::g_main_squares[si].cy;
            int d2 = dx*dx + dy*dy;
            if (d2 < best_d2) { best_d2 = d2; lock_dir = Centerline::g_main_squares[si].lock_dir; }
        }
        
        bool is_dir_locked = (lock_dir != 0);
        NarrowPassage::CorridorClass cls = NarrowPassage::g_zone_cls[z];
        int density = (z < (int)Centerline::g_zone24_density.size()) ? Centerline::g_zone24_density[z] : 0;
        bool is_dead_end = Centerline::is_dead_end_zone(z);
        
        // Don't control dead-end zones → corner deadlock avoidance
        if (is_dead_end && ENABLE_DEAD_END_PROTECTION) continue;
        
        if (cls == NarrowPassage::SUPER_NARROW || cls == NarrowPassage::NARROW) {
            // SN/N: direction lock → LV3_FULL
            if (is_dir_locked) {
                g_zone_level[z] = LV2_HARD;
                g_zone_control_dir[z] = lock_dir;
            }
        }
        else if (cls == NarrowPassage::MODERATE) {
            // M: density ≥ 5 + locked → LV1_SOFT
            if (is_dir_locked && density >= CONTROL_M_THRESHOLD) {
                g_zone_level[z] = LV1_SOFT;
                g_zone_control_dir[z] = lock_dir;
            }
        }
        else if (cls == NarrowPassage::WIDE) {
            // W: density ≥ 10 + locked → LV2_HARD
            if (is_dir_locked && density >= CONTROL_W_THRESHOLD) {
                g_zone_level[z] = LV2_HARD;
                g_zone_control_dir[z] = lock_dir;
            }
        }
    }
    
    // 检查lock_dir传播是否到达死角
    for (int z = 0; z < nz; z++) {
        if (g_zone_level[z] >= LV1_SOFT && Centerline::is_dead_end_zone(z)) {
            g_lock_reached_dead_end = true;
            g_zone_level[z] = LV0_FREE; // 死角zone不管控
            g_zone_control_dir[z] = 0;
        }
    }
}

// ====== 每tick: agent管控决策 ======
inline void update(SharedEnvironment* env) {
    if ((int)g_agent_in_control.size() != env->num_of_agents) {
        g_agent_in_control.assign(env->num_of_agents, false);
        g_agent_push_target.assign(env->num_of_agents, -1);
    }
    
    int n24c = Centerline::g_n24c;
    
    // Step 1: 更新管控级别
    update_control_levels(env);
    
    // Step 2: 对每个agent做"反-推"决策
    for (int a = 0; a < env->num_of_agents; a++) {
        int a_loc = env->curr_states[a].location;
        if (a_loc < 0) { g_agent_in_control[a] = false; g_agent_push_target[a] = -1; continue; }
        
        int a_r = a_loc / env->cols, a_c = a_loc % env->cols;
        int a_zid = (a_r / 24) * n24c + (a_c / 24);
        
        ControlLevel cl = (a_zid >= 0 && a_zid < (int)g_zone_level.size()) ? g_zone_level[a_zid] : LV0_FREE;
        int control_dir = (a_zid >= 0 && a_zid < (int)g_zone_control_dir.size()) ? g_zone_control_dir[a_zid] : 0;
        
        if (cl >= LV1_SOFT && control_dir != 0) {
            // Zone in control. Check if agent is going reverse
            bool reverse = is_going_reverse(env, a, a_loc, control_dir, a_zid);
            if (reverse) {
                g_agent_in_control[a] = true;
                // Set push target: the exit of this zone along lock_dir
                g_agent_push_target[a] = calc_push_target(env, a_loc, a_zid, control_dir);
            } else {
                g_agent_in_control[a] = false;
                g_agent_push_target[a] = -1;
            }
        } else {
            g_agent_in_control[a] = false;
            g_agent_push_target[a] = -1;
        }
    }
    
    // Step 3: 同步到PIBT
    for (int a = 0; a < env->num_of_agents && a < PIBT_Lock::MAX_AGENTS; a++) {
        PIBT_Lock::g_cold_goals[a] = g_agent_push_target[a];
        PIBT_Lock::g_agent_bound_lock[a] = g_agent_in_control[a] ? 1 : -1;
    }
}

} // namespace ControlZone
