#include "executor_heat_lock.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <unordered_map>

std::unordered_map<int, std::array<float, 4>> dir_cnt;
std::unordered_map<int, int> lock_dir;
std::unordered_map<int, int> lock_dir_prev;  // 用于打破平局

void heat_lock_init() {
    dir_cnt.clear();
    lock_dir.clear();
    lock_dir_prev.clear();
}

void heat_lock_update(SharedEnvironment* env) {
    if (!env) return;
    int cols = env->cols;
    int total_cells = env->rows * cols;
    int moves[4] = {1, cols, -1, -cols};

    dir_cnt.clear();
    lock_dir.clear();

    // 1. 预判N步算流量
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int sim_loc = env->system_states[i].location;
        int sim_orient = env->system_states[i].orientation;
        auto& actions = env->staged_actions[i];
        int steps_taken = 0;
        for (size_t step = 0; step < actions.size() && steps_taken < HEAT_LOCK_PREDICT_STEPS; step++) {
            Action a = actions[step];
            if (a == Action::FW) {
                int next_loc = sim_loc + moves[sim_orient];
                if (next_loc >= 0 && next_loc < total_cells && !env->map[next_loc]) {
                    int entering_dir = (sim_orient + 2) % 4;
                    dir_cnt[next_loc][entering_dir] += 1.0f;
                    sim_loc = next_loc;
                    steps_taken++;
                } else break;
            } else if (a == Action::CR) {
                sim_orient = (sim_orient + 1) % 4;
            } else if (a == Action::CCR) {
                sim_orient = (sim_orient - 1 + 4) % 4;
            }
        }
    }

    // 2. 初锁：总流量 >= TH则锁最多人方向
    for (auto& kv : dir_cnt) {
        int loc = kv.first;
        float de = kv.second[0], ds = kv.second[1], dw = kv.second[2], dn = kv.second[3];
        float total = de + ds + dw + dn;
        if (total < HEAT_LOCK_THRESHOLD) continue;
        float max_h = de; int max_d = 0;
        if (ds > max_h) { max_h = ds; max_d = 1; }
        if (dw > max_h) { max_h = dw; max_d = 2; }
        if (dn > max_h) { max_h = dn; max_d = 3; }
        // 打破平局：用上tick方向
        auto prev = lock_dir_prev.find(loc);
        if (prev != lock_dir_prev.end() && prev->second != -1) {
            int pd = prev->second;
            if (kv.second[pd] == max_h) max_d = pd; // 同流量用上次方向
        }
        lock_dir[loc] = (total >= HEAT_LOCK_THRESHOLD) ? max_d : -1;
    }

    // 2.5 只保留双向冲突的锁（两个方向都有人走才锁）
    for (auto& kv : lock_dir) {
        if (kv.second == -1) continue;
        auto it = dir_cnt.find(kv.first);
        if (it == dir_cnt.end()) { kv.second = -1; continue; }
        float de = it->second[0], ds = it->second[1], dw = it->second[2], dn = it->second[3];
        // 检查对轴是否双向都有流量
        bool e_w_both = (de >= 1.0f && dw >= 1.0f);
        bool n_s_both = (ds >= 1.0f && dn >= 1.0f);
        if (!e_w_both && !n_s_both) {
            kv.second = -1; // 单向流不锁
        }
    }

    // 3. 传播统一：相邻格同轴方向冲突时，统一为流量大的方向
    if (!lock_dir.empty()) {
        std::unordered_map<int, int> lock_dir_new = lock_dir;
        for (auto& kv : lock_dir) {
            int loc = kv.first;
            int ldir = kv.second;
            for (int d = 0; d < 4; d++) {
                int nloc = loc + moves[d];
                auto it = lock_dir.find(nloc);
                if (it == lock_dir.end() || it->second == -1) continue;
                if (it->second == ldir) continue;
                if ((ldir % 2) != (it->second % 2)) continue; // 不同轴不管
                
                // 统一为总流量大的格的方向
                float tot_loc = 0, tot_nloc = 0;
                auto dl = dir_cnt.find(loc);
                auto dn = dir_cnt.find(nloc);
                if (dl != dir_cnt.end()) tot_loc = dl->second[0]+dl->second[1]+dl->second[2]+dl->second[3];
                if (dn != dir_cnt.end()) tot_nloc = dn->second[0]+dn->second[1]+dn->second[2]+dn->second[3];
                int winner = (tot_loc >= tot_nloc) ? ldir : it->second;
                lock_dir_new[loc] = winner;
                lock_dir_new[nloc] = winner;
            }
        }
        lock_dir = lock_dir_new;
    }

    // 不保留上tick方向，每tick重算
    lock_dir_prev = lock_dir;

    // ---- CSV调试输出 ----
    int sys_ts = env->system_timestep;
    static FILE* fp = nullptr;
    if (!fp) {
        fp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\flow_data.csv", "w");
        if (fp) fprintf(fp, "tick,cells,avg,max,ge1,ge2,ge3,ge4,ge5,ge10,ge20,locked\n");
    }
    if (fp && sys_ts > 0 && sys_ts % 100 == 0) {
        float max_t = 0, sum_t = 0;
        int c = 0, bkt[7] = {0}, locked = 0;
        int bth[7] = {1,2,3,4,5,10,20};
        for (auto& kv : dir_cnt) {
            float t = kv.second[0]+kv.second[1]+kv.second[2]+kv.second[3];
            if (t > max_t) max_t = t;
            sum_t += t; c++;
            for (int b=0; b<7; b++) if (t >= bth[b]) bkt[b]++;
        }
        for (auto& kv : lock_dir) if (kv.second != -1) locked++;
        fprintf(fp, "%d,%d,%.1f,%.1f,%d,%d,%d,%d,%d,%d,%d,%d\n",
            sys_ts, c, c>0?sum_t/c:0, max_t,
            bkt[0],bkt[1],bkt[2],bkt[3],bkt[4],bkt[5],bkt[6],locked);
        // 前5个锁格详情
        int printed = 0;
        for (auto& kv : lock_dir) {
            if (kv.second == -1) continue;
            auto it = dir_cnt.find(kv.first);
            if (it == dir_cnt.end()) continue;
            float de=it->second[0], ds=it->second[1], dw=it->second[2], dn=it->second[3];
            fprintf(fp, "LOCK@%d:loc=%d ldir=%d(E=%.1f S=%.1f W=%.1f N=%.1f tot=%.1f)\n",
                sys_ts, kv.first, kv.second, de, ds, dw, dn, de+ds+dw+dn);
            if (++printed >= 5) break;
        }
        // 冲突检测
        int conflict = 0;
        for (auto& kv : lock_dir) {
            if (kv.second == -1) continue;
            for (int d = 0; d < 4; d++) {
                int nloc = kv.first + moves[d];
                auto it = lock_dir.find(nloc);
                if (it != lock_dir.end() && it->second != -1 && it->second != kv.second
                    && (kv.second % 2) == (it->second % 2)) {
                    conflict++;
                    if (conflict <= 3)
                        fprintf(fp, "CONF@%d:loc=%d(ldir=%d) vs nloc=%d(ldir=%d)\n",
                            sys_ts, kv.first, kv.second, nloc, it->second);
                }
            }
        }
        if (conflict > 0) fprintf(fp, "CONF@%d:total=%d\n", sys_ts, conflict/2);
        // 死锁诊断：tick 1800打印所有agent
        if (sys_ts == 1800) {
            for (int i = 0; i < (int)env->system_states.size(); i++) {
                int loc = env->system_states[i].location;
                int orient = env->system_states[i].orientation;
                auto& actions = env->staged_actions[i];
                bool fw = (!actions.empty() && actions.front() == Action::FW);
                int target = fw ? (loc + moves[orient]) : -1;
                bool locked = false;
                if (fw && target >= 0 && target < total_cells && !env->map[target]) {
                    auto it = lock_dir.find(target);
                    if (it != lock_dir.end() && it->second != -1) {
                        int opp = DIR_OPP[it->second];
                        int entering = (orient + 2) % 4;
                        locked = (entering == opp);
                    }
                }
                if (fw)
                    fprintf(fp, "DIAG@%d:a=%d loc=%d ori=%d tgt=%d locked=%d\n",
                        sys_ts, i, loc, orient, target, locked?1:0);
            }
        }
        fflush(fp);
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

    // 只检目标格逆行
    int fut_loc = curr_loc + moves[curr_orient];
    if (fut_loc >= 0 && fut_loc < total_cells && !env->map[fut_loc]) {
        auto it = lock_dir.find(fut_loc);
        if (it != lock_dir.end() && it->second != -1) {
            int lock_d = it->second;
            int opp_dir = DIR_OPP[lock_d];
            int entering_dir = (curr_orient + 2) % 4;
            if (entering_dir == opp_dir) return true;
        }
    }
    return false;
}
