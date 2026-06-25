// Backup: executor 方向锁最稳版本
// 配置: 第一歩 FW 热度 + 第一歩 mcp + 备份豁免 + TH=12
// 效果: Random ~250/500, Maze +25%, Room +6%

// --- Config ---
// STEP_DECAY = 0.98f (未使用，留作参考)
// HEAT_DECAY = 0.9f
// LOCK_THRESHOLD = 12.0f

// --- Heat 计算（第一歩 FW）---
// dir_cnt.clear();
// lock_dir.clear();
// int moves[4] = {1, env->cols, -1, -env->cols};
// for (int i = 0; i < env->system_states.size(); i++) {
//     if (env->staged_actions[i].empty()) continue;
//     if (env->staged_actions[i].front() != Action::FW) continue;
//     int curr_loc = env->system_states[i].location;
//     int orient = env->system_states[i].orientation;
//     int next_loc = curr_loc + moves[orient];
//     if (next_loc >= 0 && next_loc < env->rows * env->cols && !env->map[next_loc]) {
//         int entering_dir = (orient + 2) % 4;
//         dir_cnt[next_loc][entering_dir] += 1.0f;
//     }
// }
// // 热度更新 + 锁决策
// for (auto& kv : dir_cnt) {
//     int loc = kv.first;
//     float de = kv.second[0], ds = kv.second[1], dw = kv.second[2], dn = kv.second[3];
//     auto& h = heat[loc];
//     float he = h[0] = h[0] * HEAT_DECAY + de;
//     float hs = h[1] = h[1] * HEAT_DECAY + ds;
//     float hw = h[2] = h[2] * HEAT_DECAY + dw;
//     float hn = h[3] = h[3] * HEAT_DECAY + dn;
//     float max_h = he; int max_d = 0;
//     if (hs > max_h) { max_h = hs; max_d = 1; }
//     if (hw > max_h) { max_h = hw; max_d = 2; }
//     if (hn > max_h) { max_h = hn; max_d = 3; }
//     int lk = (max_h >= LOCK_THRESHOLD) ? max_d : -1;
//     lock_dir[loc] = lk;
// }

// --- mcp 方向锁（第一歩 + 备份豁免）---
// int curr_location = env->system_states[agent_id].location;
// int curr_orientation = env->system_states[agent_id].orientation;
// int next_location = curr_location + moves[curr_orientation];
// // 方向锁检查
// {
//     auto it = lock_dir.find(next_location);
//     if (it != lock_dir.end() && it->second != -1) {
//         int lock_d = it->second;
//         int opp_dir = (lock_d + 2) % 4;
//         int entering_dir = (curr_orientation + 2) % 4;
//         if (entering_dir == opp_dir) {
//             // 逆行，但检查是否被推（后面有车在等）
//             bool has_backup = (tpg[curr_location].size() > 1);
//             if (!has_backup) {
//                 agent_command[agent_id] = ExecutionCommand::STOP;
//                 return false;
//             }
//         }
//     }
// }
// // 原有 TPG 检查
// if (tpg[next_location].empty() || tpg[next_location].front() == agent_id) {
//     agent_command[agent_id] = ExecutionCommand::GO;
//     return true;
// } else {
//     agent_command[agent_id] = ExecutionCommand::STOP;
//     return false;
// }
