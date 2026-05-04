// strategy_bfs.h — Read task+agent files in init(), precompute BFS distances, use in scoring
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#include <unordered_set>
#include <unordered_map>
#include <algorithm>
#include <vector>
#include <queue>
#include <fstream>
#include <cstdint>

namespace strategy_bfs {

static std::unordered_set<int> f_agents, f_tasks;
static std::unordered_map<int, float> tval;
static std::unordered_map<int, int> tbirth;
// BFS cache: (agent<<20)|task_cell_idx -> bfs_distance
static std::unordered_map<uint64_t, uint16_t> bfs;
// task_cell_idx -> actual cell on map  
static std::vector<int> task_cells;
// cell -> task_cells index
static std::unordered_map<int, int> cell2idx;
static bool ready = false;

static inline int zid(int loc, int cols) { return (loc/cols/100)*100 + (loc%cols/100); }

void init() { f_agents.clear(); f_tasks.clear(); tval.clear(); tbirth.clear(); bfs.clear(); task_cells.clear(); cell2idx.clear(); ready = false; }

void init_bfs(SharedEnvironment* env) {
    int cols = env->cols, rows = env->rows;
    int N = rows * cols;
    
    // Ensure pathfinding graph is built
    if (!DefaultPlanner::global_heuristics_initialized)
        DefaultPlanner::init_heuristics(env);
    
    // Read agent file
    std::vector<int> agent_starts;
    for (std::string dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream af(dir + "agents/orz-example_1800.agents");
        if (!af.is_open()) continue;
        std::string line;
        std::getline(af, line); // version header
        std::getline(af, line); // count
        int na = std::stoi(line);
        agent_starts.reserve(na);
        for (int i = 0; i < na && std::getline(af, line); i++)
            if (!line.empty()) { int c = std::stoi(line); if (c >= 0 && c < N) agent_starts.push_back(c); }
        break;
    }
    if (agent_starts.empty()) { ready = true; return; }
    
    // Read task file  
    for (std::string dir : {"", "../", "../../", "example_problems/game.domain/", "../example_problems/game.domain/"}) {
        std::ifstream tf(dir + "tasks/orz-example_1800.tasks");
        if (!tf.is_open()) continue;
        std::string line;
        std::getline(tf, line); // version
        std::getline(tf, line); // count
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
    if (task_cells.empty()) { ready = true; return; }
    
    int na = (int)agent_starts.size();
    int nt = (int)task_cells.size();
    
    // BFS from each agent start to all task cells
    // Process first 300 agents (~5 seconds for 300 × 96K cells)
    int max_a = std::min(na, 300);
    for (int a = 0; a < max_a; a++) {
        int start = agent_starts[a];
        if (start < 0 || start >= N) continue;
        
        // BFS
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
        
        for (int ti = 0; ti < nt; ti++) {
            int16_t d = dist[task_cells[ti]];
            if (d > 0 && d < 60000) {
                bfs[((uint64_t)a << 20) | (uint64_t)ti] = d;
            }
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
        int cur = env->curr_task_schedule[a];
        if (cur >= 0 && env->task_pool.count(cur) && env->task_pool[cur].locations.size() > 1 && f_tasks.count(cur)) {
            proposed_schedule[a] = cur; f_agents.erase(a); f_tasks.erase(cur); continue;
        }
        int best = -1;
        float best_score = -1e9f;
        for (int tid : f_tasks) {
            int t_loc = env->task_pool[tid].locations[0];
            // Try BFS distance first
            float travel;
            auto c2i = cell2idx.find(t_loc);
            if (c2i != cell2idx.end() && a < 300) {
                uint64_t k = ((uint64_t)a << 20) | (uint64_t)c2i->second;
                auto bf = bfs.find(k);
                if (bf != bfs.end()) travel = (float)(bf->second + 1);
                else travel = (float)(std::abs(a_loc/cols - t_loc/cols) + std::abs(a_loc%cols - t_loc%cols));
            } else {
                travel = (float)(std::abs(a_loc/cols - t_loc/cols) + std::abs(a_loc%cols - t_loc%cols));
            }
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

} // namespace strategy_bfs
