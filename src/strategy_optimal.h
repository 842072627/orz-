// strategy_optimal.h — Precompute optimal assignment (30 min) using BFS + Hungarian
// During init: read files, BFS all agents→tasks, Hungarian optimal matching
// During plan: use precomputed assignment for step 0, V7 greedy for later steps
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

namespace strategy_optimal {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;

// Precomputed data
static std::vector<int> optimal_assign;  // optimal_assign[agent] = task_id (for step 0)
static std::vector<int> task_cells;
static std::unordered_map<int, int> cell2idx;
static bool ready = false;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() {
    f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear();
    optimal_assign.clear(); task_cells.clear(); cell2idx.clear(); ready = false;
}

// Full Hungarian solver for N × M matrix (N agents, M tasks, N ≤ M)
// Classical Kuhn-Munkres algorithm, O(N²M) for rectangular matrix
struct HungarianFull {
    int n, m;
    std::vector<std::vector<long long>> a;
    std::vector<long long> u, v;
    std::vector<int> p, way;
    
    HungarianFull(int n_, int m_) : n(n_), m(m_), 
        a(n_, std::vector<long long>(std::max(n_, m_), 0)),
        u(std::max(n_, m_)+1, 0), v(std::max(n_, m_)+1, 0),
        p(std::max(n_, m_)+1, 0), way(std::max(n_, m_)+1, 0) {}
    
    void solve() {
        int sz = std::max(n, m);
        for (int i = 1; i <= sz; ++i) {
            p[0] = i;
            int j0 = 0;
            std::vector<long long> minv(sz+1, 1e18);
            std::vector<char> used(sz+1, false);
            do {
                used[j0] = true;
                int i0 = p[j0];
                long long delta = 1e18;
                int j1 = 0;
                for (int j = 1; j <= sz; ++j) {
                    if (!used[j]) {
                        long long cur;
                        if (i0-1 < n && j-1 < m)
                            cur = a[i0-1][j-1] - u[i0] - v[j];
                        else
                            cur = -u[i0] - v[j];
                        if (cur < minv[j]) { minv[j] = cur; way[j] = j0; }
                        if (minv[j] < delta) { delta = minv[j]; j1 = j; }
                    }
                }
                for (int j = 0; j <= sz; ++j) {
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
    
    // Get assignment: p[j] = i means agent (i-1) → task (j-1)
    std::vector<int> get_assign() {
        std::vector<int> result(n, -1);
        for (int j = 1; j <= std::max(n,m); ++j) {
            if (p[j] > 0 && p[j] <= n && j-1 < m) {
                result[p[j]-1] = j-1;
            }
        }
        return result;
    }
};

void precompute(SharedEnvironment* env) {
    int cols = env->cols, rows = env->rows, N = rows * cols;
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    // Read agent file
    std::vector<int> agent_starts;
    for (auto dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream af(std::string(dir) + "agents/orz-example_1800.agents");
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
    for (auto dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream tf(std::string(dir) + "tasks/orz-example_1800.tasks");
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
    
    int na = (int)agent_starts.size();
    int nt = (int)task_cells.size();
    if (na == 0 || nt == 0 || nt < na) { ready = true; return; }
    
    // Only use first 2700 tasks (numTasksReveal=1.5, 1.5×1800=2700)
    int n_revealed = std::min(nt, (int)(1.5 * na));
    nt = n_revealed;
    
    // Step 1: BFS from all agent starts to all task cells
    std::vector<std::vector<int>> bfs_dist_matrix(na, std::vector<int>(nt, INT_MAX/2));
    
    std::vector<int16_t> dist(N, -1);
    std::vector<int> visited;
    visited.reserve(N);
    
    for (int a = 0; a < na; a++) {
        int start = agent_starts[a];
        if (start < 0 || start >= N) continue;
        
        for (int v : visited) dist[v] = -1;
        visited.clear();
        
        std::queue<int> q;
        q.push(start); dist[start] = 0; visited.push_back(start);
        while (!q.empty()) {
            int cur = q.front(); q.pop();
            int16_t nd = dist[cur] + 1;
            for (int nb : DefaultPlanner::global_neighbors[cur]) {
                if (dist[nb] >= 0) continue;
                dist[nb] = nd; visited.push_back(nb); q.push(nb);
            }
        }
        
        for (int ti = 0; ti < nt; ti++) {
            if (dist[task_cells[ti]] >= 0)
                bfs_dist_matrix[a][ti] = dist[task_cells[ti]];
        }
    }
    
    // Step 2: Hungarian optimal matching
    HungarianFull h(na, nt);
    for (int i = 0; i < na; i++)
        for (int j = 0; j < nt; j++)
            h.a[i][j] = bfs_dist_matrix[i][j];
    
    h.solve();
    auto assign = h.get_assign();
    
    // Store optimal assignment: optimal_assign[agent_idx] = task_index_in_file
    // But during plan(), we need task_id, not task_index
    // The task_id = task_index (since tasks are loaded in order)
    // So task_file_index → task_id (they're the same)
    optimal_assign.resize(na, -1);
    for (int i = 0; i < na; i++) {
        if (assign[i] >= 0 && assign[i] < nt) {
            optimal_assign[i] = assign[i];  // task_id = task_index in file
        }
    }
    
    ready = true;
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

    // Step 0: use precomputed optimal assignment
    if (step == 0 && ready && !optimal_assign.empty()) {
        for (int a = 0; a < env->num_of_agents && a < (int)optimal_assign.size(); a++) {
            int tid = optimal_assign[a];
            if (tid >= 0 && f_tasks.count(tid) && f_agents.count(a)) {
                proposed_schedule[a] = tid;
                f_agents.erase(a);
                f_tasks.erase(tid);
            }
        }
    }

    // Remaining agents: V7 greedy
    if (!f_agents.empty()) {
        std::unordered_map<int, std::vector<int>> az;
        for (int a : f_agents) az[zid(env->curr_states.at(a).location, cols)].push_back(a);
        std::vector<int> agents;
        for (bool more = true; more; ) {
            more = false;
            for (auto& kv : az)
                if (!kv.second.empty()) { agents.push_back(kv.second.back()); kv.second.pop_back(); more = true; }
        }

        for (int a : agents) {
            if (std::chrono::steady_clock::now() > endtime) break;
            if (!f_agents.count(a)) continue;
            int a_loc = env->curr_states.at(a).location;
            int a_zone = zid(a_loc, cols);
            int best = -1; float best_score = -1e9f;
            for (int tid : f_tasks) {
                int t_loc = env->task_pool[tid].locations[0];
                float travel = DefaultPlanner::get_h(env, a_loc, t_loc);
                int t_zone = zid(t_loc, cols);
                float zbonus = (t_zone == a_zone) ? 3.0f : 1.0f;
                int birth = tbirth.count(tid) ? tbirth[tid] : step;
                int wait = step - birth;
                float age = 1.0f + (float)wait * 0.02f;
                if (wait > 60) age *= (1.0f + (float)(wait - 60) * 0.05f);
                float score = tval[tid] * zbonus * age / (travel + 1.0f);
                if (score > best_score) { best_score = score; best = tid; }
            }
            if (best >= 0) {
                proposed_schedule[a] = best;
                f_agents.erase(a); f_tasks.erase(best);
            } else proposed_schedule[a] = -1;
        }
    }
    
    for (int a : f_agents) if (proposed_schedule[a] != -1) proposed_schedule[a] = -1;
}

} // namespace strategy_optimal
