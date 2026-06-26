#include "executor_heat_lock.h"
#include "executor_params.h"
#include <cmath>
#include <cstdio>
#include <cassert>
#include <unordered_map>
#include <algorithm>
#include <utility>

std::unordered_map<int, std::array<float, 4>> dir_cnt;
std::unordered_map<int, int> lock_dir;
std::unordered_map<int, int> lock_dir_prev;  // 用于打破平局

// 历史流量EMA：每格每方向的指数移动平均（α=0.1，≈20tick窗口）
std::unordered_map<int, std::array<float, 4>> flow_ema;
constexpr float FLOW_EMA_ALPHA = 0.1f;

void heat_lock_init() {
    dir_cnt.clear();
    lock_dir.clear();
    lock_dir_prev.clear();
}

void heat_lock_update(SharedEnvironment* env, const std::vector<std::vector<Action>>& staged_actions) {
    if (!env) return;
    
    auto params = get_executor_params(env->map_name);
    int predict_steps = params.predict_steps;
    float threshold = params.threshold;
    float decay_rate = params.decay_rate;
    
    int cols = env->cols;
    int total_cells = env->rows * cols;
    int moves[4] = {1, cols, -1, -cols};

    dir_cnt.clear();
    lock_dir.clear();

    // 1. 预判N步算流量（带衰减）+ 逐timestep重叠检测
    // 逐timestep occupancy: cell -> [t0, t1, ..., t11]
    std::unordered_map<int, std::array<int, 12>> plan_step_occ;
    
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int sim_loc = env->system_states[i].location;
        int sim_orient = env->system_states[i].orientation;
        auto& actions = staged_actions[i];
        int steps_taken = 0;
        for (size_t step = 0; step < actions.size() && steps_taken < predict_steps; step++) {
            Action a = actions[step];
            if (a == Action::FW) {
                int next_loc = sim_loc + moves[sim_orient];
                if (next_loc >= 0 && next_loc < total_cells && !env->map[next_loc]) {
                    float weight = decay_weight(steps_taken, decay_rate);
                    int entering_dir = (sim_orient + 2) % 4;
                    dir_cnt[next_loc][entering_dir] += weight;
                    plan_step_occ[next_loc][steps_taken]++;  // per-timestep count
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
    
    // ---- 诊断：逐timestep重叠检测 ----
    {
        int sys_ts = env->system_timestep;
        static FILE* ov_fp = nullptr;
        if (!ov_fp) {
            ov_fp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\overlap_diag.csv", "w");
            if (ov_fp) fprintf(ov_fp, "tick,cell,timestep,count\n");
        }
        if (ov_fp && sys_ts > 0) {
            for (auto& kv : plan_step_occ) {
                int loc = kv.first;
                for (int t = 0; t < predict_steps; t++) {
                    int c = kv.second[t];
                    if (c >= 2)  // 2+ agents at same timestep = overlap
                        fprintf(ov_fp, "%d,%d,%d,%d\n", sys_ts, loc, t, c);
                }
            }
            fflush(ov_fp);
        }
    }

    // 2. 初锁：总流量 >= TH则锁最多人方向
    for (auto& kv : dir_cnt) {
        int loc = kv.first;
        float de = kv.second[0], ds = kv.second[1], dw = kv.second[2], dn = kv.second[3];
        float total = de + ds + dw + dn;
        if (total < threshold) continue;
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
        lock_dir[loc] = (total >= threshold) ? max_d : -1;
    }

    // 2.5 只保留双向冲突的锁（两个方向都有人走才锁）
    if (params.bidirectional_filter) {
        for (auto& kv : lock_dir) {
            if (kv.second == -1) continue;
            auto it = dir_cnt.find(kv.first);
            if (it == dir_cnt.end()) { kv.second = -1; continue; }
            float de = it->second[0], ds = it->second[1], dw = it->second[2], dn = it->second[3];
            bool e_w_both = (de >= 0.01f && dw >= 0.01f);
            bool n_s_both = (ds >= 0.01f && dn >= 0.01f);
            if (!e_w_both && !n_s_both) {
                kv.second = -1; // 单向流不锁
            }
        }
    }

    // 3. 传播统一：相邻格同轴方向冲突时，统一为流量大的方向
    if (params.propagation && !lock_dir.empty()) {
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

    // ---- 热力图：每200tick输出agent密度分布 ----
    if (sys_ts > 0 && sys_ts % 200 == 0) {
        static FILE* hfp = nullptr;
        if (!hfp) {
            hfp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\heatmap.csv", "w");
            if (hfp) fprintf(hfp, "tick,bx,by,count\n");
        }
        if (hfp) {
            int cols = env->cols;
            int BLOCK = 30; // 30x30格一块
            int nx = (cols + BLOCK - 1) / BLOCK;
            int ny = (env->rows + BLOCK - 1) / BLOCK;
            std::vector<int> grid(nx * ny, 0);
            for (int i = 0; i < (int)env->system_states.size(); i++) {
                int loc = env->system_states[i].location;
                int bx = (loc % cols) / BLOCK;
                int by = (loc / cols) / BLOCK;
                if (bx >= 0 && bx < nx && by >= 0 && by < ny) grid[by * nx + bx]++;
            }
            for (int by = 0; by < ny; by++) {
                for (int bx = 0; bx < nx; bx++) {
                    int c = grid[by * nx + bx];
                    if (c > 0)
                        fprintf(hfp, "%d,%d,%d,%d\n", sys_ts, bx, by, c);
                }
            }
            fflush(hfp);
        }
    }

    // ---- 拥堵诊断：每200tick分析agent等待时间和阻塞点 ----
    if (sys_ts > 0 && sys_ts % 200 == 0) {
        static FILE* dfp = nullptr;
        static std::vector<int> prev_loc;
        static std::vector<int> stuck_ticks;
        
        // 初始化跟踪数组
        int na = (int)env->system_states.size();
        if ((int)prev_loc.size() != na) {
            prev_loc.resize(na, -1);
            stuck_ticks.resize(na, 0);
        }
        
        if (!dfp) {
            dfp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\diagnostics.csv", "w");
            if (dfp) fprintf(dfp, "tick,free_agents,stuck_agents,avg_wait,top5_congested,top5_stuck\n");
        }
        
        if (dfp) {
            // 更新等待时间
            int stuck_cnt = 0;
            float sum_wait = 0;
            int wait_cnt = 0;
            for (int i = 0; i < na; i++) {
                int loc = env->system_states[i].location;
                if (prev_loc[i] != -1 && prev_loc[i] == loc) {
                    stuck_ticks[i]++;
                } else {
                    stuck_ticks[i] = 0;
                }
                prev_loc[i] = loc;
                if (stuck_ticks[i] > 0) {
                    sum_wait += stuck_ticks[i];
                    wait_cnt++;
                }
                if (stuck_ticks[i] >= 50) stuck_cnt++;
            }
            
            // 统计每格agent数（找最拥堵的格）
            std::unordered_map<int, int> cell_pop;
            for (int i = 0; i < na; i++) {
                int loc = env->system_states[i].location;
                cell_pop[loc]++;
            }
            
            // 找Top5拥堵格
            std::vector<std::pair<int,int>> sorted;
            for (auto& kv : cell_pop) sorted.push_back(kv);
            std::sort(sorted.begin(), sorted.end(), 
                [](auto& a, auto& b) { return a.second > b.second; });
            
            int top5_cong = 0, top5_stuck_track = stuck_cnt;
            for (int i = 0; i < (int)sorted.size() && i < 5; i++) {
                if (sorted[i].second >= 3) top5_cong++;
            }
            
            // 统计"自由"agent（不堵的）
            int free_agents = 0;
            for (int i = 0; i < na; i++) {
                if (stuck_ticks[i] == 0) free_agents++;
            }
            
            fprintf(dfp, "%d,%d,%d,%.1f,%d,%d\n",
                sys_ts, free_agents, stuck_cnt,
                wait_cnt > 0 ? sum_wait / wait_cnt : 0,
                top5_cong, top5_stuck_track);
            
            // 前5拥堵格详情
            int printed = 0;
            int cols = env->cols;
            for (auto& p : sorted) {
                if (p.second < 3) break;
                int bx = (p.first % cols) / 30;
                int by = (p.first / cols) / 30;
                fprintf(dfp, "CONGEST@%d:loc=%d block=(%d,%d) agents=%d\n",
                    sys_ts, p.first, bx, by, p.second);
                if (++printed >= 5) break;
            }
            
            fflush(dfp);
        }
    }
}

// ---- 实时occupancy方向锁 ----
// 基于当前每格实际agent数，不依赖plan预测
void heat_lock_update_realtime(SharedEnvironment* env) {
    if (!env) return;
    auto params = get_executor_params(env->map_name);
    
    int cols = env->cols;
    int total_cells = env->rows * cols;
    int moves[4] = {1, cols, -1, -cols};

    dir_cnt.clear();
    lock_dir.clear();

    // 1. 统计每格agent数量和他们面对的方向
    std::unordered_map<int, int> cell_occ;  // cells with actual agents
    std::unordered_map<int, std::array<int, 4>> real_dir; // actual direction counts
    
    for (int i = 0; i < (int)env->system_states.size(); i++) {
        int loc = env->system_states[i].location;
        int orient = env->system_states[i].orientation;
        cell_occ[loc]++;
        real_dir[loc][orient]++;
    }

    // 2. 锁：每格≥3个agent才锁，锁最多人面对的方向
    // 或者连续2格的corridor也有锁（总人数≥3的2格走廊）
    for (auto& kv : cell_occ) {
        int loc = kv.first;
        int occ = kv.second;
        if (occ < 2) continue;  // ≥2个agent同格就锁
        
        // 算这一格的总方向
        auto& dirs = real_dir[loc];
        int max_d = 0;
        for (int d = 1; d < 4; d++) {
            if (dirs[d] > dirs[max_d]) max_d = d;
        }
        lock_dir[loc] = max_d;
    }

    // 3. 对撞检测：相邻格各有≥1个agent对走时锁
    for (auto& kv : cell_occ) {
        if (kv.second < 1) continue;
        int loc = kv.first;
        for (int d = 0; d < 4; d++) {
            int nloc = loc + moves[d];
            auto it = cell_occ.find(nloc);
            if (it == cell_occ.end() || it->second < 1) continue;
            // 两个相邻格都有≥2个agent
            // 检查是否有对撞方向
            auto& dirs_a = real_dir[loc];
            auto& dirs_b = real_dir[nloc];
            // 如果A格的主要方向指向B格，B格的主要方向指向A格→对撞
            int max_a = 0;
            for (int dd = 1; dd < 4; dd++) if (dirs_a[dd] > dirs_a[max_a]) max_a = dd;
            int max_b = 0;
            for (int dd = 1; dd < 4; dd++) if (dirs_b[dd] > dirs_b[max_b]) max_b = dd;
            
            if ((d == max_a && DIR_OPP[d] == max_b) || (DIR_OPP[d] == max_a && d == max_b)) {
                // 对撞！锁更多人方向
                int tot_a = kv.second;
                int tot_b = it->second;
                if (tot_a >= tot_b)
                    lock_dir[loc] = max_a;
                else
                    lock_dir[nloc] = max_b;
            }
        }
    }

    // 4. 传播
    if (params.propagation && !lock_dir.empty()) {
        std::unordered_map<int, int> lock_dir_new = lock_dir;
        for (auto& kv : lock_dir) {
            int loc = kv.first;
            int ldir = kv.second;
            for (int d = 0; d < 4; d++) {
                int nloc = loc + moves[d];
                auto it = lock_dir.find(nloc);
                if (it == lock_dir.end() || it->second == -1) continue;
                if (it->second == ldir) continue;
                if ((ldir % 2) != (it->second % 2)) continue;
                int tot_loc = cell_occ[loc];
                int tot_nloc = cell_occ[nloc];
                int winner = (tot_loc >= tot_nloc) ? ldir : it->second;
                lock_dir_new[loc] = winner;
                lock_dir_new[nloc] = winner;
            }
        }
        lock_dir = lock_dir_new;
    }

    // ---- CSV: 每tick记录锁统计 ----
    {
        int sys_ts = env->system_timestep;
        static FILE* rfp = nullptr;
        if (!rfp) {
            rfp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\realtime_flow.csv", "w");
            if (rfp) fprintf(rfp, "tick,cells_with_agents,locked,max_per_cell\n");
        }
        if (rfp && sys_ts > 0 && sys_ts % 100 == 0) {
            int max_occ = 0;
            for (auto& kv : cell_occ) if (kv.second > max_occ) max_occ = kv.second;
            int locked_cnt = 0;
            for (auto& kv : lock_dir) if (kv.second != -1) locked_cnt++;
            fprintf(rfp, "%d,%zu,%d,%d\n", sys_ts, cell_occ.size(), locked_cnt, max_occ);
            // 锁详情
            int printed = 0;
            for (auto& kv : lock_dir) {
                if (kv.second == -1) continue;
                auto it = cell_occ.find(kv.first);
                int occ = (it != cell_occ.end()) ? it->second : 0;
                auto& dirs = real_dir[kv.first];
                fprintf(rfp, "LOCK@%d:loc=%d ldir=%d occ=%d (E=%d S=%d W=%d N=%d)\n",
                    sys_ts, kv.first, kv.second, occ,
                    dirs[0], dirs[1], dirs[2], dirs[3]);
                if (++printed >= 5) break;
            }
            fflush(rfp);
        }
    }
    
    // ---- 热力图：每200tick输出agent密度分布 ----
    {
        int sys_ts = env->system_timestep;
        if (sys_ts > 0 && sys_ts % 200 == 0) {
            static FILE* hfp = nullptr;
            if (!hfp) {
                hfp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\heatmap.csv", "w");
                if (hfp) fprintf(hfp, "tick,bx,by,count\n");
            }
            if (hfp) {
                int BLOCK = 30;
                int nx = (cols + BLOCK - 1) / BLOCK;
                int ny = (env->rows + BLOCK - 1) / BLOCK;
                std::vector<int> grid(nx * ny, 0);
                for (auto& kv : cell_occ) {
                    int loc = kv.first;
                    int bx = (loc % cols) / BLOCK;
                    int by = (loc / cols) / BLOCK;
                    if (bx >= 0 && bx < nx && by >= 0 && by < ny) grid[by * nx + bx] += kv.second;
                }
                for (int by = 0; by < ny; by++)
                    for (int bx = 0; bx < nx; bx++)
                        if (grid[by * nx + bx] > 0)
                            fprintf(hfp, "%d,%d,%d,%d\n", sys_ts, bx, by, grid[by * nx + bx]);
                fflush(hfp);
            }
        }
    }
}

// ---- 历史流量EMA方向锁 ----
// 基于agent实际位置+方向的指数移动平均，不依赖plan
// ---- Stall-based方向锁 ----
// 追踪agent在同一格的等待时间，累加为格的"拥堵权重"
// 权重高 = 这个格长期有agent在等 = 堵了 → 锁主流方向
void heat_lock_update_historical(SharedEnvironment* env) {
    if (!env) return;
    auto params = get_executor_params(env->map_name);
    if (params.use_realtime) return;
    
    int moves[4] = {1, env->cols, -1, -env->cols};
    int sys_ts = env->system_timestep;
    
    // 1. 追踪每个agent的持续等待时间
    static std::vector<int> stall_prev_loc;
    static std::vector<int> stall_ticks;
    int na = (int)env->system_states.size();
    if ((int)stall_prev_loc.size() != na) {
        stall_prev_loc.resize(na, -1);
        stall_ticks.resize(na, 0);
    }
    
    // 每格的总拥堵权重
    std::unordered_map<int, int> stall_weight;  // cell → sum of stall ticks
    std::unordered_map<int, std::array<int, 4>> stall_dir;  // cell → direction counts of stalled agents
    std::unordered_map<int, int> stall_agent_count;  // cell → number of unique stalled agents
    
    for (int i = 0; i < na; i++) {
        int loc = env->system_states[i].location;
        int orient = env->system_states[i].orientation;
        
        if (stall_prev_loc[i] != -1 && stall_prev_loc[i] == loc) {
            stall_ticks[i]++;  // same location = waiting
        } else {
            stall_ticks[i] = 0;  // moved = reset
        }
        stall_prev_loc[i] = loc;
        
        if (stall_ticks[i] > 0) {
            stall_weight[loc] += stall_ticks[i];
            stall_dir[loc][orient]++;
            stall_agent_count[loc]++;
        }
    }

    // 2. 锁决策
    float stall_th = (params.ema_threshold > 0.01f) ? params.ema_threshold : 5.0f;
    lock_dir.clear();
    
    for (auto& kv : stall_weight) {
        int loc = kv.first;
        int weight = kv.second;  // total stall ticks for this cell
        int agents = stall_agent_count[loc];
        
        // 需要足够的拥堵才能锁
        // weight ≥ STALL_TH 意味着有agent在此连续等待
        if (weight < stall_th) continue;
        
        // 锁最拥堵的agent面对的方向
        auto& dirs = stall_dir[loc];
        int max_d = 0;
        for (int d = 1; d < 4; d++) if (dirs[d] > dirs[max_d]) max_d = d;
        
        // 但不要锁向墙的方向
        int target = loc + moves[max_d];
        if (target < 0 || target >= env->rows * env->cols || env->map[target])
            continue;  // 锁的方向是墙，不锁
        
        lock_dir[loc] = max_d;
    }
    lock_dir_prev = lock_dir;

    // 3. 传播
    if (params.propagation && !lock_dir.empty()) {
        std::unordered_map<int, int> lock_dir_new = lock_dir;
        for (auto& kv : lock_dir) {
            int loc = kv.first;
            int ldir = kv.second;
            for (int d = 0; d < 4; d++) {
                int nloc = loc + moves[d];
                auto it = lock_dir.find(nloc);
                if (it == lock_dir.end() || it->second == -1 || it->second == ldir) continue;
                if ((ldir % 2) != (it->second % 2)) continue;
                int w_loc = stall_weight[loc];
                int w_nloc = stall_weight[nloc];
                int winner = (w_loc >= w_nloc) ? ldir : it->second;
                lock_dir_new[loc] = winner;
                lock_dir_new[nloc] = winner;
            }
        }
        lock_dir = lock_dir_new;
    }

    // 4. 诊断输出
    if (sys_ts > 0 && sys_ts % 200 == 0) {
        static FILE* sfp = nullptr;
        if (!sfp) {
            sfp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\stall_data.csv", "w");
            if (sfp) fprintf(sfp, "tick,avg_stall,max_stall,cells_stalled,locked,agents_stalled\n");
        }
        if (sfp) {
            int max_w = 0, stalled_cells = 0, locked = 0, stalled_agents = 0;
            float sum_w = 0;
            for (auto& kv : stall_weight) {
                if (kv.second > max_w) max_w = kv.second;
                sum_w += kv.second; stalled_cells++;
            }
            for (auto& kv : lock_dir) if (kv.second != -1) locked++;
            for (auto s : stall_ticks) if (s > 0) stalled_agents++;
            
            fprintf(sfp, "%d,%.1f,%d,%d,%d,%d\n",
                sys_ts, stalled_cells>0?sum_w/stalled_cells:0, max_w,
                stalled_cells, locked, stalled_agents);
            
            int printed = 0;
            for (auto& kv : lock_dir) {
                if (kv.second == -1) continue;
                fprintf(sfp, "STALLLOCK@%d:loc=%d ldir=%d weight=%d agents=%d\n",
                    sys_ts, kv.first, kv.second,
                    stall_weight[kv.first], stall_agent_count[kv.first]);
                if (++printed >= 5) break;
            }
            fflush(sfp);
        }
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

// 链式反应诊断
void heat_lock_diag_hotspots(SharedEnvironment* env) {
    if (!env) return;
    int sys_ts = env->system_timestep;
    if (sys_ts <= 0 || sys_ts % 50 != 0) return;  // 每50tick采样
    
    int cols = env->cols;
    int na = (int)env->system_states.size();
    
    // 热点区域: 中心坐标 + 半径(格)
    struct HotSpot { int x, y, radius; const char* name; };
    HotSpot spots[] = {
        {624, 488, 30, "hot1_624_488"},
        {610, 436, 30, "hot2_610_436"},
        {696, 194, 30, "hot3_696_194"},
    };
    
    static FILE* diag_fp = nullptr;
    if (!diag_fp) {
        diag_fp = fopen("C:\\Users\\Administrator\\Desktop\\test-studio-mapf\\big-final-test\\executor-data\\chain_diag.csv", "w");
        if (diag_fp) fprintf(diag_fp, "tick,zone,total_agents,stuck_agents,direction_eb,dir_wb,dir_nb,dir_sb,headon_pairs,max_stall\n");
    }
    if (!diag_fp) return;
    
    // 用于检测对撞：追踪每格的agent
    std::unordered_map<int, std::vector<std::pair<int,int>>> cell_agents; // cell -> (agent_id, orientation)
    
    for (int i = 0; i < na; i++) {
        int loc = env->system_states[i].location;
        int orient = env->system_states[i].orientation;
        cell_agents[loc].push_back({i, orient});
    }
    
    // 追踪每个agent的stall时间
    static std::vector<int> diag_prev_loc;
    static std::vector<int> diag_stall;
    if ((int)diag_prev_loc.size() != na) {
        diag_prev_loc.resize(na, -1);
        diag_stall.resize(na, 0);
    }
    
    for (int i = 0; i < na; i++) {
        int loc = env->system_states[i].location;
        if (diag_prev_loc[i] == loc) diag_stall[i]++;
        else diag_stall[i] = 0;
        diag_prev_loc[i] = loc;
    }
    
    for (auto& spot : spots) {
        int cx = spot.x, cy = spot.y, r = spot.radius;
        int r2 = r * r;
        int zone_agents = 0, zone_stuck = 0;
        int dir_e = 0, dir_w = 0, dir_n = 0, dir_s = 0;
        int headon = 0, max_stall = 0;
        
        for (int i = 0; i < na; i++) {
            int loc = env->system_states[i].location;
            int px = loc % cols;
            int py = loc / cols;
            int dx = px - cx, dy = py - cy;
            if (dx*dx + dy*dy > r2) continue;
            
            zone_agents++;
            if (diag_stall[i] > 0) zone_stuck++;
            if (diag_stall[i] > max_stall) max_stall = diag_stall[i];
            
            int orient = env->system_states[i].orientation;
            if (orient == 0) dir_e++;
            else if (orient == 2) dir_w++;
            else if (orient == 3) dir_n++;
            else if (orient == 1) dir_s++;
        }
        
        // 检测对撞：一个格里有2+个agent且面向相反
        for (auto& kv : cell_agents) {
            int loc = kv.first;
            int px = loc % cols, py = loc / cols;
            int dx = px - cx, dy = py - cy;
            if (dx*dx + dy*dy > r2) continue;
            
            auto& agents = kv.second;
            if (agents.size() < 2) continue;
            // 检查是否有对向
            bool has_e = false, has_w = false, has_n = false, has_s = false;
            for (auto& a : agents) {
                int o = a.second;
                if (o == 0) has_e = true;
                if (o == 2) has_w = true;
                if (o == 3) has_n = true;
                if (o == 1) has_s = true;
            }
            if ((has_e && has_w) || (has_n && has_s)) headon++;
        }
        
        fprintf(diag_fp, "%d,%s,%d,%d,%d,%d,%d,%d,%d,%d\n",
            sys_ts, spot.name, zone_agents, zone_stuck,
            dir_e, dir_w, dir_n, dir_s, headon, max_stall);
    }
    fflush(diag_fp);
}
