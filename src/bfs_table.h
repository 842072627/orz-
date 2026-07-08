// bfs_table.h -- BFS distance matrix (No Man Sky approach)
// 
// Two modes (config switch):
//   USE_BFS_TABLE=0 (default): off-main component check for branch exemption, single fast BFS
//   USE_BFS_TABLE=1            full BFS distance matrix, multi-threaded, O(1) dist lookup
//
#pragma once
#include "config.h"
#include "SharedEnv.h"
#include <vector>
#include <queue>
#include <thread>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <climits>
#include <chrono>
#include <atomic>

#ifndef USE_BFS_TABLE
#define USE_BFS_TABLE 0  // default off (local dev); set to 1 for cloud
#endif

namespace BFSTable {

// ====== Always-available: off-main components ======
static bool g_offmain_ready = false;
static std::vector<int> g_offmain_comp;     // [loc] = component (-2=main, -1=wall, >=0=offmain)
static int g_offmain_comp_count = 0;

// ====== Full matrix (only when USE_BFS_TABLE=1) ======
static bool g_full_ready = false;
static std::vector<std::vector<uint16_t>> g_dist_rows; // [valid_idx][valid_idx] = dist
static std::vector<int> g_cell_to_idx;                  // [map_pos] = valid_idx or -1
static std::vector<int> g_idx_to_cell;                  // [valid_idx] = map_pos
static int g_valid_count = 0;
static int g_cols = 0, g_rows = 0, g_total = 0;

// ====== Phase 1: Common init (always runs) ======
inline void init_common(const SharedEnvironment* env,
                        const std::vector<int>& zone_is_main, int n24c) {
    g_rows = env->rows;
    g_cols = env->cols;
    g_total = g_rows * g_cols;

    // Cell indexing
    g_cell_to_idx.assign(g_total, -1);
    g_idx_to_cell.clear();
    g_valid_count = 0;
    for (int i = 0; i < g_total; i++) {
        if (env->map[i] == 0) {
            g_cell_to_idx[i] = g_valid_count++;
            g_idx_to_cell.push_back(i);
        }
    }

    // Main zone mask
    std::vector<char> cell_in_main(g_total, 0);
    for (int i = 0; i < g_total; i++) {
        if (env->map[i] != 0) continue;
        int zid = (i / g_cols / 24) * n24c + (i % g_cols / 24);
        if (zid >= 0 && zid < (int)zone_is_main.size() && zone_is_main[zid])
            cell_in_main[i] = 1;
    }

    // Single BFS for off-main components
    g_offmain_comp.assign(g_total, -1);
    g_offmain_comp_count = 0;
    {
        std::queue<int> q;
        for (int start = 0; start < g_total; start++) {
            if (env->map[start] != 0) continue;
            if (g_offmain_comp[start] >= 0) continue;
            if (cell_in_main[start]) { g_offmain_comp[start] = -2; continue; }

            int cid = ++g_offmain_comp_count;
            q.push(start);
            g_offmain_comp[start] = cid;
            while (!q.empty()) {
                int cur = q.front(); q.pop();
                int neigh[4] = {cur - g_cols, cur + g_cols, cur - 1, cur + 1};
                for (int n : neigh) {
                    if (n < 0 || n >= g_total) continue;
                    if (env->map[n] != 0) continue;
                    if (g_offmain_comp[n] >= 0) continue;
                    if (cell_in_main[n]) { g_offmain_comp[n] = -2; continue; }
                    g_offmain_comp[n] = cid;
                    q.push(n);
                }
            }
        }
    }

    int off_cnt = 0, main_cnt = 0;
    for (int i = 0; i < g_total; i++) {
        if (g_offmain_comp[i] > 0) off_cnt++;
        else if (g_offmain_comp[i] == -2) main_cnt++;
    }
#if USE_DIAG_LOG
    printf("BFSTABLE: offmain ready - %d comps, %d main-zone, %d offmain cells\n",
           g_offmain_comp_count, main_cnt, off_cnt);
#endif
    g_offmain_ready = true;
}

// ====== Branch exemption (always available) ======
inline bool is_branch_path(int loc_a, int loc_b) {
    if (!g_offmain_ready) return false;
    int ca = g_offmain_comp[loc_a];
    int cb = g_offmain_comp[loc_b];
    return (ca >= 0 && cb >= 0 && ca == cb);
}

// ====== Distance lookup ======
inline int get_dist(int loc_a, int loc_b) {
    if (g_full_ready) {
        int ai = g_cell_to_idx[loc_a];
        int bi = g_cell_to_idx[loc_b];
        if (ai >= 0 && bi >= 0 && (size_t)bi < g_dist_rows[ai].size()) {
            uint16_t d = g_dist_rows[ai][bi];
            if (d != (uint16_t)-1) return (int)d;
        }
    }
    return -1;  // caller falls back
}

// ====== Phase 2: Full BFS distance matrix (USE_BFS_TABLE=1 only) ======
// Each row is a separate vector (~192KB per row for 97K cells)
// Cache-friendly: BFS from one source only touches its own 192KB row
inline void build_full(const SharedEnvironment* env) {
    if (!g_offmain_ready || g_valid_count == 0) return;

    size_t mb_total = (size_t)g_valid_count * g_valid_count * 2 / (1024 * 1024);
#if USE_DIAG_LOG
    printf("BFSTABLE: building full matrix %d x %d = %zu entries (~%zu MB), each row ~%zu KB\n",
           g_valid_count, g_valid_count,
           (size_t)g_valid_count * g_valid_count,
           mb_total,
           (size_t)g_valid_count * 2 / 1024);
#endif

    // Allocate per-row vectors (No Man Sky approach: matrix[target][source])
    g_dist_rows.resize(g_valid_count);
    // Pre-allocate all rows
    for (int i = 0; i < g_valid_count; i++) {
        g_dist_rows[i].assign(g_valid_count, (uint16_t)-1);
    }

    auto start_time = std::chrono::steady_clock::now();
    int time_limit_ms = 120000;

    unsigned int n_threads = std::thread::hardware_concurrency();
    if (n_threads < 1) n_threads = 4;
    if (n_threads > 12) n_threads = 12;

    std::atomic<bool> g_timeout{false};

    auto do_work = [&](int thr_id, int num_threads) {
        // per-thread BFS queue (reused across source rows)
        std::vector<int> bfs_q;
        bfs_q.reserve(g_valid_count);

        for (int src_idx = thr_id; src_idx < g_valid_count && !g_timeout.load(); src_idx += num_threads) {
            // Time check every 16 sources
            if ((src_idx & 15) == (unsigned int)thr_id % 16) {
                auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - start_time).count();
                if (elapsed >= time_limit_ms) { g_timeout.store(true); break; }
            }

            uint16_t* row = g_dist_rows[src_idx].data();
            int src_loc = g_idx_to_cell[src_idx];
            row[src_idx] = 0;
            bfs_q.clear();
            bfs_q.push_back(src_loc);

            size_t head = 0;
            while (head < bfs_q.size()) {
                int cur = bfs_q[head++];
                uint16_t nd = row[g_cell_to_idx[cur]] + 1;

                // 4-neighbor unrolled
                int n, nidx;

                n = cur - g_cols;
                if (n >= 0) {
                    nidx = g_cell_to_idx[n];
                    if (nidx >= 0 && row[nidx] == (uint16_t)-1) { row[nidx] = nd; bfs_q.push_back(n); }
                }

                n = cur + g_cols;
                if (n < g_total) {
                    nidx = g_cell_to_idx[n];
                    if (nidx >= 0 && row[nidx] == (uint16_t)-1) { row[nidx] = nd; bfs_q.push_back(n); }
                }

                n = cur - 1;
                if (cur % g_cols > 0) {
                    nidx = g_cell_to_idx[n];
                    if (nidx >= 0 && row[nidx] == (uint16_t)-1) { row[nidx] = nd; bfs_q.push_back(n); }
                }

                n = cur + 1;
                if (cur % g_cols < g_cols - 1) {
                    nidx = g_cell_to_idx[n];
                    if (nidx >= 0 && row[nidx] == (uint16_t)-1) { row[nidx] = nd; bfs_q.push_back(n); }
                }
            }
        }
    };

    std::vector<std::thread> threads(n_threads);
    for (unsigned int t = 0; t < n_threads; t++)
        threads[t] = std::thread(do_work, (int)t, (int)n_threads);
    for (unsigned int t = 0; t < n_threads; t++)
        threads[t].join();

    int built = 0;
    for (int i = 0; i < g_valid_count; i++)
        if (g_dist_rows[i][i] == 0) built++;

    if (built > 0) {
        g_full_ready = true;
#if USE_DIAG_LOG
        printf("BFSTABLE: full matrix %s - %d/%d sources (%zu MB, ~%.1fs)\n",
               g_timeout.load() ? "TIMEOUT" : "COMPLETE",
               built, g_valid_count, mb_total,
               std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count());
#endif
    } else {
#if USE_DIAG_LOG
        printf("BFSTABLE: full matrix FAILED (0 sources built)\n");
#endif
        g_dist_rows.clear();
        g_dist_rows.shrink_to_fit();
    }
}

// ====== Top-level init ======
inline void init(const SharedEnvironment* env,
                 const std::vector<int>& zone_is_main, int n24c) {
    init_common(env, zone_is_main, n24c);
#if USE_BFS_TABLE
    build_full(env);
#else
#if USE_DIAG_LOG
    printf("BFSTABLE: full matrix disabled (USE_BFS_TABLE=0)\n");
#endif
#endif
}

} // namespace BFSTable
