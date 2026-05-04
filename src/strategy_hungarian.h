// strategy_hungarian.h — BFS distances + Hungarian algorithm for optimal agent-task matching
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <climits>
#include <cstdint>

namespace strategy_hungarian {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

// BFS distances from file-reading (init phase)
static std::unordered_map<uint64_t, uint16_t> bfs;
static std::vector<int> task_cells, agent_starts;
static std::unordered_map<int, int> cell2idx;
static bool ready = false;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

// --- Hungarian Algorithm (for small N) ---
// Classic Kuhn-Munkres for minimization. Handles N ≤ 100.
struct Hungarian {
    int n;
    std::vector<std::vector<int>> a; // cost matrix
    std::vector<int> u, v, p, way;
    
    Hungarian(int n_) : n(n_), a(n_, std::vector<int>(n_, 0)), u(n_+1, 0), v(n_+1, 0), p(n_+1, 0), way(n_+1, 0) {}
    
    void solve() {
        for (int i = 1; i <= n; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<int> minv(n+1, INT_MAX);
            std::vector<char> used(n+1, false);
            do {
                used[j0] = true;
                int i0 = p[j0], delta = INT_MAX, j1 = 0;
                for (int j = 1; j <= n; ++j) {
                    if (!used[j]) {
                        int cur = a[i0-1][j-1] - u[i0] - v[j];
                        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                    }
                }
                for (int j = 0; j <= n; ++j) {
                    if (used[j]) { u[p[j]] += delta; v[j] -= delta; }
                    else { minv[j] -= delta; }
                }
                j0 = j1;
            } while (p[j0] != 0);
            do {
                int j1 = way[j0];
                p[j0] = p[j1];
                j0 = j1;
            } while (j0);
        }
    }
    
    // Get assignment: agent_idx -> task_idx (both 0-based)
    std::vector<int> get_assignment(int num_agents, int num_tasks) {
        std::vector<int> assign(num_agents, -1);
        for (int j = 1; j <= n; ++j) {
            if (p[j] > 0 && p[j] <= num_agents && j-1 < num_tasks) {
                assign[p[j]-1] = j-1;
            }
        }
        return assign;
    }
};

// --- file reading + BFS ---
void init_bfs(SharedEnvironment* env) {
    int cols = env->cols, rows = env->rows, N = rows * cols;
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    // Read agent file
    for (std::string dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream af(dir + "agents/orz-example_1800.agents");
        if (!af.is_open()) continue;
        std::string line;
        std::getline(af, line);
        std::getline(af, line);
        int na = std::stoi(line);
        agent_starts.reserve(na);
        for (int i = 0; i < na && std::getline(af, line); i++)
            if (!line.empty()) { int c = std::stoi(line); if (c >= 0 && c < N) agent_starts.push_back(c); }
        break;
    }
    
    // Read task file
    for (std::string dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream tf(dir + "tasks/orz-example_1800.tasks");
        if (!tf.is_open()) continue;
        std::string line;
        std::getline(tf, line);
        std::getline(tf, line);
        int nt = std::stoi(line);
        task_cells.reserve(nt);
        for (int i = 0; i < nt && std::getline(tf, line); i++) {
            if (line.empty()) continue;
            auto cp = line.find(',');
            if (cp != std::string::npos) {
                int c = std::stoi(line.substr(0, cp));
                if (c >= 0 && c < N) {
                    cell2idx[c] = (int)task_cells.size();
                    task_cells.push_back(c);
                }
            }
        }
        break;
    }
    
    if (agent_starts.empty() || task_cells.empty()) { ready = true; return; }
    
    // BFS from first 300 agents
    int max_a = std::min((int)agent_starts.size(), 300);
    for (int a = 0; a < max_a; a++) {
        int start = agent_starts[a];
        if (start < 0 || start >= N) continue;
        std::queue<int> q;
        std::vector<int16_t> dist(N, -1);
        q.push(start); dist[start] = 0;
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int16_t nd = dist[cur] + 1;
            for (int nb : DefaultPlanner::global_neighbors[cur]) {
                if (dist[nb] >= 0) continue;
                dist[nb] = nd; q.push(nb);
            }
        }
        for (int ti = 0; ti < (int)task_cells.size(); ti++) {
            int16_t d = dist[task_cells[ti]];
            if (d > 0 && d < 60000) bfs[((uint64_t)a << 20) | (uint64_t)ti] = d;
        }
    }
    ready = true;
}

// Get distance (BFS or Manhattan)
static inline float get_dist(int agent, int a_loc, int t_loc, int cols) {
    auto c2i = cell2idx.find(t_loc);
    if (c2i != cell2idx.end() && agent < 300) {
        uint64_t k = ((uint64_t)agent << 20) | (uint64_t)c2i->second;
        auto bf = bfs.find(k);
        if (bf != bfs.end()) return (float)(bf->second + 1);
    }
    return (float)(std::abs(a_loc/cols - t_loc/cols) + std::abs(a_loc%cols - t_loc%cols));
}

void init() {
    f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear();
    bfs.clear(); task_cells.clear(); agent_starts.clear(); cell2idx.clear(); ready = false;
}

void plan(int time_limit, SharedEnvironment* env, std::vector<int>& proposed_schedule) {
    int limit = time_limit/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE;
    TimePoint endtime = std::chrono::steady_clock::now() + std::chrono::milliseconds(limit);
    int cols = env->cols, step = env->curr_timestep;

    for (int tid : env->new_tasks) {
        if (!tbirth.count(tid)) tbirth[tid] = step;
        float idist = 0;
        auto& locs = env->task_pool[tid].locations;
        for (size_t i = 1; i < locs.size(); i++)
            idist += DefaultPlanner::get_h(env, locs[i-1], locs[i]);
        int n = (int)locs.size();
        float base = (idist > 0) ? (float)n/idist : 10.0f;
        tval[tid] = (n == 1) ? base * 1.5f : base;
    }

    f_agents.insert(env->new_freeagents.begin(), env->new_freeagents.end());
    f_tasks.insert(env->new_tasks.begin(), env->new_tasks.end());
    for (int a = 0; a < env->num_of_agents; a++)
        if (env->curr_task_schedule[a] < 0) f_agents.insert(a);
    if (f_agents.empty() || f_tasks.empty()) {
        for (int a : f_agents) proposed_schedule[a] = -1;
        return;
    }

    std::vector<int> agents(f_agents.begin(), f_agents.end());
    
    // HUNGARIAN: process ALL free agents in batches for optimal matching
    int batch_sz = 200;
    for (int b = 0; b < (int)agents.size() && !f_agents.empty() && !f_tasks.empty(); b += batch_sz) {
        int b_end = std::min(b + batch_sz, (int)agents.size());
        int na = b_end - b;
        if (na < 1) continue;
        if (na == 1) { // Single agent: greedy
            int a = agents[b];
            int a_loc = env->curr_states.at(a).location;
            int a_zone = zid(a_loc, cols);
            int best = -1; float best_score = -1e9f;
            for (int tid : f_tasks) {
                int t_loc = env->task_pool[tid].locations[0];
                float travel = get_dist(a, a_loc, t_loc, cols);
                int t_zone = zid(t_loc, cols);
                float zbonus = (t_zone == a_zone) ? 3.0f : 1.0f;
                int birth = tbirth.count(tid) ? tbirth[tid] : step;
                int wait = step - birth;
                float age = 1.0f + (float)wait * 0.02f;
                if (wait > 60) age *= (1.0f + (float)(wait - 60) * 0.05f);
                float score = tval[tid] * zbonus * age / (travel + 1.0f);
                if (score > best_score) { best_score = score; best = tid; }
            }
            if (best >= 0) { proposed_schedule[a] = best; f_agents.erase(a); f_tasks.erase(best); }
            else proposed_schedule[a] = -1;
            continue;
        }
        if ((int)f_tasks.size() < na) continue;
        int nt = std::min((int)f_tasks.size(), na * 2);  // Consider top 2× agents tasks
        
        // Build task list (first nt tasks from f_tasks)
        std::vector<int> task_list;
        int cnt = 0;
        for (int tid : f_tasks) {
            if (cnt >= nt) break;
            task_list.push_back(tid);
            cnt++;
        }
        
        // Build cost matrix: na × nt (pad to square with dummy cols if needed)
        int n_sq = std::max(na, nt);
        Hungarian h(n_sq);
        for (int i = 0; i < na; i++) {
            int a = agents[b + i];
            int a_loc = env->curr_states.at(a).location;
            for (int j = 0; j < nt; j++) {
                int tid = task_list[j];
                int t_loc = env->task_pool[tid].locations[0];
                h.a[i][j] = (int)get_dist(a, a_loc, t_loc, cols);
            }
            // Fill dummy columns with high cost
            for (int j = nt; j < n_sq; j++) h.a[i][j] = 999999;
        }
        // Fill dummy rows with high cost
        for (int i = na; i < n_sq; i++)
            for (int j = 0; j < n_sq; j++) h.a[i][j] = 999999;
        
        h.solve();
        auto assign = h.get_assignment(na, nt);
        
        // Apply assignments
        for (int i = 0; i < na; i++) {
            if (assign[i] >= 0 && assign[i] < nt) {
                int a = agents[b + i];
                int tid = task_list[assign[i]];
                proposed_schedule[a] = tid;
                f_agents.erase(a);
                f_tasks.erase(tid);
            }
        }
    }
    
    // Remaining unassigned
    for (int a : f_agents) proposed_schedule[a] = -1;
}

} // namespace strategy_hungarian
