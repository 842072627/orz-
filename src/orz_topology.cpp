#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <cstdio>
#include "../inc/SharedEnv.h"
// === ORZ TOPOLOGY (A391+) - Bridge detection + dead-end area capacity limiting ===
int g_orz_cols_t = 0;
std::vector<int> orz_dead_ends_t;
std::unordered_map<int,int> orz_component_of_t;
std::unordered_map<int,std::vector<int>> orz_bridge_subtrees; // bridge-connected dead-end regions
std::unordered_set<int> orz_dead_nodes_t; // all nodes in dead-end regions
std::unordered_set<int> orz_bridge_nodes_t; // nodes that are bridges
bool orz_topo_t = false;

inline int orz_valid_t(SharedEnvironment* env, int loc) {
    return (loc >= 0 && loc < (int)env->map.size() && env->map[loc] == 0);
}
int orz_nbr_t(SharedEnvironment* env, int loc, int* out) {
    int c = loc % g_orz_cols_t, r = loc / g_orz_cols_t, n = 0;
    if (c > 0 && orz_valid_t(env, loc-1)) out[n++] = loc-1;
    if (c < g_orz_cols_t-1 && orz_valid_t(env, loc+1)) out[n++] = loc+1;
    if (r > 0 && orz_valid_t(env, loc-g_orz_cols_t)) out[n++] = loc-g_orz_cols_t;
    if (r < env->rows-1 && orz_valid_t(env, loc+g_orz_cols_t)) out[n++] = loc+g_orz_cols_t;
    return n;
}
// Count dead-end neighbors (degree-1 neighbors)
int orz_dead_nbr_count(SharedEnvironment* env, int loc, int* nbr) {
    int nn = orz_nbr_t(env, loc, nbr);
    int cnt = 0;
    for (int i = 0; i < nn; i++) if (orz_dead_nodes_t.count(nbr[i])) cnt++;
    return cnt;
}
void analyze_orz_topo(SharedEnvironment* env) {
    if (orz_topo_t) return;
    g_orz_cols_t = env->cols;
    int rows = env->rows, cols = env->cols, N = rows * cols;
    std::vector<char> vis(N, 0);
    std::vector<int> free_locs, nb(4);
    for (int i = 0; i < N; i++) if (env->map[i] == 0) free_locs.push_back(i);
    
    // Step 1: Find all dead-end nodes (degree=1)
    for (int loc : free_locs) {
        int nn = orz_nbr_t(env, loc, nb.data());
        if (nn == 1) {
            orz_dead_ends_t.push_back(loc);
            orz_dead_nodes_t.insert(loc);
        }
    }
    // Step 2: Find bridge nodes - dead-end nodes adjacent to non-dead-end nodes
    for (int loc : free_locs) {
        int nn = orz_nbr_t(env, loc, nb.data());
        if (nn == 1) continue; // skip degree-1 nodes
        int dead_nbrs = 0;
        for (int i = 0; i < nn; i++) if (orz_dead_nodes_t.count(nb[i])) dead_nbrs++;
        if (dead_nbrs > 0) orz_bridge_nodes_t.insert(loc);
    }
    // Step 3: Build dead-end regions (BFS from each bridge node into dead-end tree)
    std::unordered_map<int,int> region_of;
    int region_id = 1; // 0 = main component
    for (int bridge : orz_bridge_nodes_t) {
        std::vector<int> q; q.reserve(128); q.push_back(bridge);
        size_t qi = 0; vis[bridge] = 1; region_of[bridge] = region_id;
        while (qi < q.size()) {
            int cur = q[qi++];
            int nn = orz_nbr_t(env, cur, nb.data());
            for (int i = 0; i < nn; i++) {
                int nb2 = nb[i];
                if (!vis[nb2] && orz_dead_nodes_t.count(nb2)) {
                    vis[nb2] = 1; region_of[nb2] = region_id; q.push_back(nb2);
                }
            }
        }
        region_id++;
    }
    orz_topo_t = true;
    fprintf(stderr, "[TOPO] ORZ: free=%zu dead=%zu bridges=%zu regions=%d\n",
            free_locs.size(), orz_dead_ends_t.size(), orz_bridge_nodes_t.size(), region_id-1);
}
