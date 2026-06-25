#include "executor_heat_lock.h"
#include <cmath>
#include <cassert>

// 数据
std::unordered_map<int, std::array<float, 4>> heat_map;
std::unordered_map<int, std::array<float, 4>> dir_cnt;
std::unordered_map<int, int> lock_dir;
std::unordered_map<int, int> backup_count; // [loc] = number of agents who will visit

void heat_lock_init() {
    heat_map.clear();
    dir_cnt.clear();
    lock_dir.clear();
    backup_count.clear();
}

void heat_lock_update(SharedEnvironment* env) {
    if (!env) return;
    int cols = env->cols;
    int total_cells = env->rows * cols;
    int moves[4] = {1, cols, -1, -cols};

    dir_cnt.clear();
    lock_dir.clear();
    backup_count.clear();

    // 全量预判 + 备份计数
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int sim_loc = env->system_states[i].location;
        backup_count[sim_loc]++; // 当前所在格
        int sim_orient = env->system_states[i].orientation;
        bool sim_valid = true;
        auto& actions = env->staged_actions[i];
        for (size_t step = 0; step < actions.size() && sim_valid; step++) {
            float w = pow(HEAT_LOCK_STEP_DECAY, (float)step);
            Action a = actions[step];
            if (a == Action::FW) {
                int next_loc = sim_loc + moves[sim_orient];
                if (next_loc >= 0 && next_loc < total_cells && !env->map[next_loc]) {
                    int entering_dir = (sim_orient + 2) % 4;
                    dir_cnt[next_loc][entering_dir] += w;
                    backup_count[next_loc]++;
                    sim_loc = next_loc;
                } else {
                    sim_valid = false;
                }
            } else if (a == Action::CR) {
                sim_orient = (sim_orient + 1) % 4;
            } else if (a == Action::CCR) {
                sim_orient = (sim_orient - 1 + 4) % 4;
            }
        }
    }

    // 热度 + 锁
    for (auto& kv : dir_cnt) {
        int loc = kv.first;
        float de = kv.second[0], ds = kv.second[1], dw = kv.second[2], dn = kv.second[3];
        auto& h = heat_map[loc];
        float total = de + ds + dw + dn;
        float he = h[0] = de;
        float hs = h[1] = ds;
        float hw = h[2] = dw;
        float hn = h[3] = dn;

        float max_h = he; int max_d = 0;
        if (hs > max_h) { max_h = hs; max_d = 1; }
        if (hw > max_h) { max_h = hw; max_d = 2; }
        if (hn > max_h) { max_h = hn; max_d = 3; }
        lock_dir[loc] = (total >= HEAT_LOCK_THRESHOLD) ? max_d : -1;
    }
}

bool heat_lock_check(int agent_id, int curr_loc, int curr_orient, SharedEnvironment* env) {
    if (!env) return false;
    if (agent_id < 0 || agent_id >= (int)env->system_states.size()) return false;
    if (env->staged_actions[agent_id].empty()) return false;
    if (env->staged_actions[agent_id].front() != Action::FW) return false;

    int cols = env->cols;
    int total_cells = env->rows * cols;
    int moves[4] = {1, cols, -1, -cols};

    // (a) 离开当前格前，检当前格锁
    auto cur_lock = lock_dir.find(curr_loc);
    if (cur_lock != lock_dir.end() && cur_lock->second != -1) {
        if (curr_orient == cur_lock->second) {
            return true; // 正逆行离开锁格
        }
    }

    // (b) 进入下一个格前，检目标格锁
    int fut_loc = curr_loc + moves[curr_orient];
    if (fut_loc >= 0 && fut_loc < total_cells && !env->map[fut_loc]) {
        auto it = lock_dir.find(fut_loc);
        if (it != lock_dir.end() && it->second != -1) {
            int lock_d = it->second;
            int opp_dir = DIR_OPP[lock_d];
            int entering_dir = (curr_orient + 2) % 4;
            if (entering_dir == opp_dir) {
                return true; // 逆行进入锁格
            }
        }
    }

    return false; // 放行
}
