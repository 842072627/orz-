// Default executor baseline implementation with ORZ direction lock.
//
// Direction lock: segment corridor by main_dist, detect E/W flow from
// planner's 15-step intended final position, lock minority direction.

#include "my_executor_orz.h"
#include "executor_heat_lock.h"
#include "orz_highway.h"
#include <queue>
#include <cmath>
#include <unordered_set>

namespace MyExecutorOrz{

    std::unordered_map<int, std::list<int>> tpg;
    std::unordered_map<int, std::list<int>> temp_tpg;

    vector<int> previous_locations;
    vector<State> predicted_states;

    int window_size;
    bool first_execution = true;

    // ── centerline data ──
    std::vector<int> cj_main_dist;   // [loc] → main axis position or -1
    std::vector<int> cj_branch_dist; // [loc] → dist from centerline or -1
    int cj_rows = 0, cj_cols = 0, cj_total = 0;
    bool cj_ready = false;

    // ── segment data ──
    struct CjSegment {
        int min_m, max_m;
        int labeled_cells;  // number of labeled cells in this segment (constant)
        int agents_inside;   // agent count on labeled cells in segment
        bool capped;         // over capacity
    };
    std::vector<CjSegment> cj_segments;
    constexpr int SEG_STEP = 100;
    constexpr int CAP_DIST = 300; // block agents this far outside segment
    constexpr int SEG_CAPACITY = 5; // agents per segment before capping

    // ── highway sign data ──
    std::vector<unsigned char> orz_sign; // [loc] → 1 if labeled
    
    // ── zone flow control ──
    std::vector<int> cell_zone; // [loc] → zone id (0=not in any zone)
    std::vector<int> zone_capacity;
    std::vector<int> zone_agent_count; // per-tick
    int num_zones = 0;

    // ── per-agent intent from plan ──
    std::vector<int> agent_intent; // +1=east, -1=west, 0=unknown/not on corridor
    // ── historical trail for intent correction ──
    std::vector<std::deque<int>> agent_trail; // last N locations per agent
    constexpr int TRAIL_LEN = 200;
    constexpr int MIN_TRAIL = 50;

    // ── project ──
    static void project(int loc, int &m, int &b) {
        if (!cj_ready || loc < 0 || loc >= (int)cj_branch_dist.size()) {
            m = -1; b = -1; return;
        }
        b = cj_branch_dist[loc];
        m = (b >= 0) ? cj_main_dist[loc] : -1;
    }

    // ── initialize: compute centerline ──
    void execute_initialize(int preprocess_time_limit, SharedEnvironment* env) {
        window_size = (int)env->min_planner_communication_time / (env->action_time*env->max_counter) + 1;
        previous_locations.resize(env->num_of_agents, -1);
        agent_intent.assign(env->num_of_agents, 0);
        agent_trail.resize(env->num_of_agents);

        // ---- build centerline (from optsvr/GameBalanceOptScheduler) ----
        cj_rows = env->rows; cj_cols = env->cols; cj_total = cj_rows * cj_cols;
        cj_main_dist.assign(cj_total, -1);
        cj_branch_dist.assign(cj_total, -1);

        // DTW (distance transform)
        std::vector<int> dtw(cj_total, INT_MAX/2);
        { std::queue<int> q;
          for (int i=0; i<cj_total; i++)
              if (env->map[i] != 0) { dtw[i]=0; q.push(i); }
          while (!q.empty()) {
              int cur=q.front(); q.pop();
              int nd=dtw[cur]+1;
              int nb[4]={cur-cj_cols, cur+cj_cols, cur-1, cur+1};
              for (int n : nb)
                  if (n>=0 && n<cj_total && env->map[n]==0 && nd<dtw[n])
                      { dtw[n]=nd; q.push(n); }
          }
        }

        // find leftmost/rightmost walkable cell
        int start_loc=-1, goal_loc=-1, best_sc=-1, best_gc=-1, min_x=cj_cols, max_x=0;
        for (int i=0; i<cj_total; i++) {
            if (env->map[i]!=0) continue;
            int c=i%cj_cols;
            if (c<min_x || (c==min_x && dtw[i]>best_sc))
                { min_x=c; start_loc=i; best_sc=dtw[i]; }
            if (c>max_x || (c==max_x && dtw[i]>best_gc))
                { max_x=c; goal_loc=i; best_gc=dtw[i]; }
        }
        if (start_loc<0 || goal_loc<0) { cj_ready=true; return; }

        // BFS shortest path
        std::vector<int> came_from(cj_total, -1);
        { std::queue<int> q; q.push(start_loc); came_from[start_loc]=start_loc;
          int found=-1;
          while (!q.empty()) {
              int cur=q.front(); q.pop();
              if (cur==goal_loc) { found=cur; break; }
              int nb[4]={cur-cj_cols, cur+cj_cols, cur-1, cur+1};
              for (int n : nb)
                  if (n>=0 && n<cj_total && env->map[n]==0 && came_from[n]==-1)
                      { came_from[n]=cur; q.push(n); }
          }
        }

        // reconstruct path
        std::vector<int> base_path;
        { int cur=goal_loc;
          while (cur!=start_loc) { base_path.push_back(cur); cur=came_from[cur]; }
          base_path.push_back(start_loc);
          std::reverse(base_path.begin(), base_path.end());
        }

        // accumulate main_dist along centerline
        std::unordered_map<int,int> cl_main;
        int cumul=0;
        cl_main[base_path[0]]=0;
        for (size_t i=1; i<base_path.size(); i++) {
            int r1=base_path[i-1]/cj_cols, c1=base_path[i-1]%cj_cols;
            int r2=base_path[i]/cj_cols,   c2=base_path[i]%cj_cols;
            cumul += (int)std::round(std::sqrt((double)((c2-c1)*(c2-c1)+(r2-r1)*(r2-r1))));
            cl_main[base_path[i]] = cumul;
        }

        // BFS outward → propagate main_dist & branch_dist
        std::queue<int> sq;
        for (size_t i=0; i<base_path.size(); i++) {
            int loc=base_path[i];
            cj_branch_dist[loc]=0;
            cj_main_dist[loc]=cl_main[loc];
            sq.push(loc);
            if (i+1 < base_path.size()) {
                int nloc=base_path[i+1];
                int r1=loc/cj_cols, c1=loc%cj_cols;
                int r2=nloc/cj_cols, c2=nloc%cj_cols;
                int m1=cl_main[loc], m2=cl_main[nloc];
                for (int k=1; k<20; k++) {
                    float t=(float)k/20.0f;
                    int ir=(int)std::round(r1+(r2-r1)*t);
                    int ic=(int)std::round(c1+(c2-c1)*t);
                    int iloc=ir*cj_cols+ic;
                    if (iloc>=0 && iloc<cj_total && env->map[iloc]==0 && cj_branch_dist[iloc]==-1) {
                        cj_branch_dist[iloc]=0;
                        cj_main_dist[iloc]=(int)std::round(m1+(m2-m1)*t);
                        sq.push(iloc);
                    }
                }
            }
        }
        while (!sq.empty()) {
            int cur=sq.front(); sq.pop();
            int nd=cj_branch_dist[cur]+1, nm=cj_main_dist[cur];
            int nb[4]={cur-cj_cols, cur+cj_cols, cur-1, cur+1};
            for (int n : nb)
                if (n>=0 && n<cj_total && env->map[n]==0 && cj_branch_dist[n]==-1)
                    { cj_branch_dist[n]=nd; cj_main_dist[n]=nm; sq.push(n); }
        }

        // init segments
        int nseg = cumul / SEG_STEP + 1;
        cj_segments.resize(nseg);
        for (int i=0; i<nseg; i++) {
            cj_segments[i].min_m = i*SEG_STEP;
            cj_segments[i].max_m = (i+1)*SEG_STEP;
            cj_segments[i].labeled_cells = 0;
            cj_segments[i].agents_inside = 0;
            cj_segments[i].capped = false;
        }

        // load highway sign data (labeled narrow corridors)
        orz_sign.assign(cj_total, 0);
        int sign_cnt = 0;
        for (int i=0; i<cj_total; i++) {
            uint8_t s = ORZ_Highway::get_sign(i);
            if (s != 0) { orz_sign[i] = 1; sign_cnt++; }
        }
        fprintf(stderr, "[DBG] orz_sign loaded: %d cells\n", sign_cnt);

        // count labeled cells per segment (constant)
        cj_ready = true; // temporarily enable project()
        for (int i=0; i<cj_total; i++) {
            if (!orz_sign[i]) continue;
            int m,b; project(i, m, b);
            if (m < 0) continue;
            int si = m / SEG_STEP;
            if (si >= 0 && si < nseg) cj_segments[si].labeled_cells++;
        }
        cj_ready = false;

        cj_ready = true;
    }

    // ── compute plan intent (simulate staged actions → main_dist diff) ──
    static int compute_plan_intent(const vector<Action>& actions, int start_loc, int start_ori, SharedEnvironment* env) {
        if (actions.empty()) return 0;
        int moves[4] = {1, env->cols, -1, -env->cols};
        int loc = start_loc;
        int ori = start_ori;
        for (size_t t=0; t<actions.size(); t++) {
            if (actions[t]==Action::FW) loc += moves[ori];
            else if (actions[t]==Action::CR) ori = (ori+1)%4;
            else if (actions[t]==Action::CCR) { ori--; if(ori<0)ori=3; }
        }
        int sm=-1,sb=-1; project(start_loc,sm,sb);
        int em=-1,eb=-1; project(loc,em,eb);
        if (sm<0 || em<0) return 0;
        return (em > sm) ? 1 : (em < sm) ? -1 : 0;
    }

    // ── process_new_plan ──
    vector<State> execute_process_new_plan(int sync_time_limit, Plan& plan_struct, vector<vector<Action>>& staged_actions, SharedEnvironment* env) {
        if (predicted_states.size() != env->num_of_agents)
            predicted_states = env->system_states;
        if (first_execution) {
            for (int i=0; i<env->num_of_agents; i++)
                if (tpg.find(env->curr_states[i].location)==tpg.end())
                    tpg[env->curr_states[i].location].push_back(i);
            first_execution = false;
        }
        for (int i=0; i<env->num_of_agents; i++)
            previous_locations[i] = env->system_states[i].location;

        auto curr_states = predicted_states;
        int moves[4] = {1, env->cols, -1, -env->cols};
        std::vector<std::vector<Action>> plan = plan_struct.actions;
        for (int timestep=0; timestep<(int)plan[0].size() && timestep<window_size; timestep++) {
            for (size_t i=0; i<plan.size(); i++) {
                int nl=curr_states[i].location, no=curr_states[i].orientation;
                if (plan[i][timestep]==Action::FW) {
                    nl += moves[curr_states[i].orientation];
                    tpg[nl].push_back(i);
                } else if (plan[i][timestep]==Action::CR) no=(curr_states[i].orientation+1)%4;
                else if (plan[i][timestep]==Action::CCR) { no=(curr_states[i].orientation-1)%4; if(no==-1)no=3; }
                curr_states[i].location=nl; curr_states[i].orientation=no; curr_states[i].timestep++;
                if (plan[i][timestep]!=Action::NA && plan[i][timestep]!=Action::W)
                    staged_actions[i].push_back(plan[i][timestep]);
            }
        }

        heat_lock_update(env, staged_actions);

        // intent from plan (15 steps)
        agent_intent.assign(env->num_of_agents, 0);
        for (int i=0; i<env->num_of_agents && i<(int)staged_actions.size(); i++) {
            agent_intent[i] = compute_plan_intent(staged_actions[i], env->system_states[i].location, env->system_states[i].orientation, env);
        }

        // MCP simulation
        vector<int> action_index(env->num_of_agents,0);
        curr_states = env->system_states; temp_tpg.clear();
        for (int t=0; t<window_size; t++) {
            auto pre_states = curr_states;
            for (int i=0; i<env->num_of_agents; i++) {
                if (staged_actions[i].size() <= action_index[i]) { curr_states[i].timestep++; continue; }
                Action ca = staged_actions[i][action_index[i]];
                int cl=curr_states[i].location, co=curr_states[i].orientation;
                int nl=cl, no=co;
                auto tpg_it=tpg.find(cl);
                if (tpg_it==tpg.end() || tpg_it->second.empty() || tpg_it->second.front()!=i) { curr_states[i].timestep++; continue; }
                if (ca==Action::FW) {
                    nl=cl+moves[co];
                    auto tit=tpg.find(nl);
                    if (tit!=tpg.end() && !tit->second.empty() && tit->second.front()==i) action_index[i]++; else nl=cl;
                } else if (ca==Action::CR) { no=(co+1)%4; action_index[i]++; }
                else if (ca==Action::CCR) { no=(co-1+4)%4; action_index[i]++; }
                curr_states[i].location=nl; curr_states[i].orientation=no;
            }
            for (int i=0; i<env->num_of_agents; i++) {
                if (t==0) temp_tpg[pre_states[i].location].push_back(i);
                if (pre_states[i].location!=curr_states[i].location) {
                    auto tpg_it2=tpg.find(pre_states[i].location);
                    if (tpg_it2!=tpg.end() && !tpg_it2->second.empty() && tpg_it2->second.front()==i) {
                        tpg_it2->second.pop_front();
                        temp_tpg[curr_states[i].location].push_back(i);
                    }
                }
            }
        }
        for (int i=0; i<env->num_of_agents; i++)
            if (action_index[i] < (int)staged_actions[i].size())
                staged_actions[i].resize(action_index[i]);
        predicted_states = curr_states;
        tpg = temp_tpg;

        // dump intent
        { int sys_ts=env->system_timestep;
          static int last_d=-100;
          if (sys_ts-last_d>=100) { last_d=sys_ts;
              int e=0,w=0; for (int i=0; i<(int)agent_intent.size(); i++) { if(agent_intent[i]>0)e++; else if(agent_intent[i]<0)w++; }
              fprintf(stderr, "[DBG] plan_intent t=%d e=%d w=%d/%d\n", sys_ts, e, w, (int)agent_intent.size());
          }
        }

        return predicted_states;
    }

    // ── every tick ──
    void execute_next_command(int exec_time_limit, std::vector<ExecutionCommand>& agent_command, SharedEnvironment* env) {
        // update TPG
        if (env->system_timestep > 0) {
            for (int i=0; i<env->system_states.size(); i++) {
                int pl=previous_locations[i], cl=env->system_states[i].location;
                auto it=tpg.find(pl);
                if (it!=tpg.end() && !it->second.empty() && it->second.front()==i) {
                    if (pl!=cl) { it->second.pop_front(); previous_locations[i]=cl; }
                } else { previous_locations[i]=cl; }
            }
        }

        // ── update agent trail ──
        for (int i=0; i<env->system_states.size(); i++) {
            agent_trail[i].push_back(env->system_states[i].location);
            if ((int)agent_trail[i].size() > TRAIL_LEN)
                agent_trail[i].pop_front();
        }

        // ── flow control: cap segments with >10 agents on labeled cells ──
        if (cj_ready) {
            for (auto& seg : cj_segments) seg.agents_inside = 0, seg.capped = false;
            for (int i=0; i<env->system_states.size(); i++) {
                int loc = env->system_states[i].location;
                if (loc < 0 || loc >= cj_total || !orz_sign[loc]) continue;
                int cm, cb; project(loc, cm, cb);
                if (cm < 0) continue;
                int si = cm / SEG_STEP;
                if (si >= 0 && si < (int)cj_segments.size())
                    cj_segments[si].agents_inside++;
            }
            constexpr int CAP_THRESH = 10;
            for (auto& seg : cj_segments)
                if (seg.labeled_cells > 0 && seg.agents_inside > CAP_THRESH)
                    seg.capped = true;

            // diagnostics
            { static int last_p=-1; if (env->system_timestep%200==0 && env->system_timestep!=last_p) { last_p=env->system_timestep;
                int nc=0, max_a=0, max_l=0;
                for (auto& s:cj_segments) { if(s.capped)nc++; if(s.agents_inside>max_a)max_a=s.agents_inside; if(s.labeled_cells>max_l)max_l=s.labeled_cells; }
                fprintf(stderr,"[DBG] t=%d capped=%d/%d max_agents=%d max_labeled=%d\n", env->system_timestep, nc, (int)cj_segments.size(), max_a, max_l);
            }}
        }

        // ── mcp ──
        for (int i=0; i<env->system_states.size(); i++)
            mcp(i, agent_command, env);

        // diagnostics (capped segments)
        { static int last_s=-1; if (env->system_timestep%200==0 && env->system_timestep!=last_s) { last_s=env->system_timestep;
            int nc=0; for (auto& s:cj_segments) if(s.capped)nc++;
            fprintf(stderr,"[DBG] t=%d capped=%d/%d\n", env->system_timestep, nc, (int)cj_segments.size());
        }}
    }
    // ── diagnostics ──
    int dir_stop_count = 0;

    // ── mcp ──
    bool mcp(int agent_id, std::vector<ExecutionCommand>& agent_command, SharedEnvironment* env) {
        if (env->staged_actions[agent_id].empty()) {
            agent_command[agent_id] = ExecutionCommand::STOP;
            return false;
        }
        if (env->staged_actions[agent_id].front() != Action::FW) {
            agent_command[agent_id] = ExecutionCommand::GO;
            return false;
        }

        int moves[4] = {1, env->cols, -1, -env->cols};
        int loc = env->system_states[agent_id].location;
        int ori = env->system_states[agent_id].orientation;
        int nloc = loc + moves[ori];
        if (nloc < 0 || nloc >= cj_total || env->map[nloc]) {
            agent_command[agent_id] = ExecutionCommand::STOP;
            return false;
        }

        // flow control: block agents close to capped segment, not on labeled cell
        if (cj_ready) {
            int nm, nb; project(nloc, nm, nb);
            int cm, cb; project(loc, cm, cb);
            if (cm < 0 || nm < 0) return true;
            int si = nm / SEG_STEP;
            if (si >= 0 && si < (int)cj_segments.size() && cj_segments[si].capped) {
                // agent on labeled cell → free
                if (orz_sign[loc]) return true;
                // check distance to segment
                int seg_center = (cj_segments[si].min_m + cj_segments[si].max_m) / 2;
                int dist = std::abs(cm - seg_center) + cb;
                if (dist <= 300) {
                    agent_command[agent_id] = ExecutionCommand::STOP;
                    return false;
                }
            }
        }

        // TPG check
        if (tpg[nloc].empty() || tpg[nloc].front() == agent_id) {
            agent_command[agent_id] = ExecutionCommand::GO;
            return true;
        }
        agent_command[agent_id] = ExecutionCommand::STOP;
        return false;
    }

}  // namespace MyExecutorOrz
