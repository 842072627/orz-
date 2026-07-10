


#include "my_search.h"
#include "../pibt_lock.h"
#include "../orz_highway.h"
#include <iostream>


namespace MyPlanner{
std::chrono::nanoseconds t;
namespace {
int g_astar_warning_count = 0;

void log_astar_warning(const char* message, int start, int goal)
{
    if (g_astar_warning_count++ < 20) {
        std::cerr << "[GameDirectionalPlanner] " << message
                  << " start=" << start << " goal=" << goal << std::endl;
    }
}
}
//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h;

    if (is_large_map(env)){
        h = manhattanDistance(start,goal,env);
    }
    else{
        if(ht.empty())
            h = manhattanDistance(start,goal,env);
        else
            h = get_heuristic(ht,env, start, ns);
    }
    

    
    s_node* root = mem.generate_node(start,0, h,0,0,0);

    if (start == goal){
        traj.clear();
        traj.push_back(start);
        return *root;
    }

    pqueue_min_of open;
    re_of re;

    open.push(root);

    int  diff, d, cost, op_flow, total_cross, all_vertex_flow,vertex_flow, depth,p_diff, p_d;
    int next_d1, next_d2, next_d1_loc, next_d2_loc;
    int temp_op, temp_vertex;
    double tie_breaker, decay_factor;

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];

#if USE_WALL_HUGGING && USE_WALL_HUGGING_ASTAR
    int corridor_dir = 0;
    if (CenterlineAccess::g_main_dist && CenterlineAccess::g_branch_dist &&
        CenterlineAccess::g_zone_is_main && CenterlineAccess::g_n24c > 0) {
        int s_zi = (start / env->cols / 24) * CenterlineAccess::g_n24c + (start % env->cols / 24);
        int g_zi = (goal / env->cols / 24) * CenterlineAccess::g_n24c + (goal % env->cols / 24);
        if (s_zi >= 0 && s_zi < (int)CenterlineAccess::g_zone_is_main->size() &&
            (*CenterlineAccess::g_zone_is_main)[s_zi] &&
            g_zi >= 0 && g_zi < (int)CenterlineAccess::g_zone_is_main->size() &&
            (*CenterlineAccess::g_zone_is_main)[g_zi] && g_zi != s_zi) {
            int s_m = (*CenterlineAccess::g_main_dist)[start];
            int g_m = (*CenterlineAccess::g_main_dist)[goal];
            if (s_m >= 0 && g_m >= 0 && g_m != s_m)
                corridor_dir = (g_m > s_m) ? 1 : -1;
        }
    }
#endif


    while (open.size() > 0){
        if (deadline != nullptr && std::chrono::steady_clock::now() >= *deadline){
            return s_node();
        }
        s_node* curr = open.pop();
        curr->close();

        if (curr->id == goal){
            goal_node = curr;
            break;
        }
        expanded++;
        getNeighborLocs(ns,neighbors,curr->id);
        
        for (int i=0; i<4; i++){
            int next = neighbors[i];
            if (next == -1){
                continue;
            }
            if (!orz_directional_move_allowed(curr->id, next, env)) {
                continue;
            }
            cost = curr->g+1;
            const int move_dir = get_d(next - curr->id, env);
            // ORZ highway guide (soft penalty)
            { uint8_t pd; if (ORZ_Highway::get_guide(curr->id, pd) && move_dir == pd) cost += 50000; }

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            if(is_large_map(env)){
                h = manhattanDistance(next,goal,env);
            }
            else{
                if(ht.empty())
                    h = manhattanDistance(next,goal,env);
                else
                    h = get_heuristic(ht,env, next, ns);
            }

            diff = next - curr->id;
            d = get_d(diff,env);
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d!=d)
                    tie_breaker = 0.1;
                else
                    tie_breaker = 0;
                //tie breaking on prefering moving forward
            }


            temp_op = ( (flow[curr->id].d[d]+1) * flow[next].d[(d+2)%4]);///( ( (flow[curr->id].d[d]+1) + flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = 1;
            for (int j=0; j<4; j++){
                temp_vertex += flow[next].d[j];                
            }

            op_flow += temp_op;
        
            all_vertex_flow+= (temp_vertex-1) /2;

#if USE_WALL_HUGGING && USE_WALL_HUGGING_ASTAR
            if (corridor_dir != 0 && CenterlineAccess::g_branch_dist) {
                int bd = (*CenterlineAccess::g_branch_dist)[next];
                int target_bd = (corridor_dir > 0) ? 99 : 0;
                int diff = (bd > target_bd) ? (bd - target_bd) : (target_bd - bd);
                if (diff > WALL_HUG_BRANCH_BIAS_MAX) diff = WALL_HUG_BRANCH_BIAS_MAX;
                op_flow += diff;
            }
#endif

            p_diff = 0;
            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
            }

            op_flow += curr->op_flow; //op_flow is contra flow
            all_vertex_flow += curr->all_vertex_flow;

            s_node temp_node(next,cost,h,op_flow, depth);
            temp_node.tie_breaker = tie_breaker;
            temp_node.set_all_flow(op_flow,  all_vertex_flow);

            if (!mem.has_node(next)){
                s_node* next_node = mem.generate_node(next,cost,h,op_flow, depth,all_vertex_flow);
                next_node->parent = curr;
                next_node->tie_breaker = tie_breaker;
                open.push(next_node);
                generated++;
            }
            else{ 
                s_node* existing = mem.get_node(next);

                if (!existing->is_closed()){
                    if (re(temp_node,*existing)){
                        existing->g = cost;
                        existing->parent = curr;
                        existing->depth = depth;
                        existing->tie_breaker = tie_breaker;
                        existing->set_all_flow(op_flow,  all_vertex_flow);
                        open.decrease_key(existing);
                    }
                }
                else{

                    if (re(temp_node,*existing)){ 
                        log_astar_warning("astar re-expansion, aborting this guide update", start, goal);
                        return s_node();
                    }

                } 
            }
        }
            

          
    }


    if (goal_node == nullptr){
        log_astar_warning("astar no directed path", start, goal);
        return s_node();
    }

    traj.resize(goal_node->depth+1);
    s_node* curr = goal_node;
    for (int i=goal_node->depth; i>=0; i--){
        traj[i] = curr->id;
        curr = curr->parent;
    }

    return *goal_node;
}
}
