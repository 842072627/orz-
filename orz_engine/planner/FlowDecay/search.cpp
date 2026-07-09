


#include "search.h"
#include "FlowVertexCost.h"
#include "RuntimeConfig.h"
#include "HeuristicMatrix.h"
#include "../../schedule/GameBalanceOpt/PibtLock.h"

#include <algorithm>

namespace OptPlan{
std::chrono::nanoseconds t;

// 统一 h 值计算（A* guide path 专用）
// 注意：A* 的状态空间是 location-only。turn_aware 矩阵的 min(4方向) >= location_only_dist
// （途中转弯使得即使最优朝向也比纯位置距离大），会过估计导致 re-expansion。
// 因此 A* 只用 location-only 矩阵查询，turn_aware 模式下回退到 lazy BFS / Manhattan。
static inline int compute_h(SharedEnvironment* env, int loc, int goal,
                            HeuristicTable& ht, Neighbors* ns) {
    auto& hm = orz_engine::heuristic_matrix();
    if (hm.is_ready() && !hm.is_turn_aware()) {
        // location-only 矩阵可直接用
        uint16_t d = hm.get(loc, goal);
        if (d != orz_engine::DIST_UNREACHABLE) return static_cast<int>(d);
    }
    // turn_aware 模式或矩阵未就绪：回退到原始逻辑
    if (is_large_map(env)) {
        return manhattanDistance(loc, goal, env);
    }
    if (ht.empty()) {
        return manhattanDistance(loc, goal, env);
    }
    return get_heuristic(ht, env, loc, ns);
}

static inline int turn_steps(int from_orientation, int to_orientation) {
    if (from_orientation < 0 || to_orientation < 0) return 0;
    from_orientation %= 4;
    to_orientation %= 4;
    const int diff = (to_orientation - from_orientation + 4) % 4;
    return std::min(diff, 4 - diff);
}

//a astar minimized the opposide traffic flow with existing traffic flow

s_node astar(SharedEnvironment* env, const std::vector<Int4>& flow,
    HeuristicTable& ht, Traj& traj,
    MemoryPool& mem, int start, int goal, Neighbors* ns, const TimePoint* deadline,
    const std::vector<std::vector<int>>* extra_weights, int start_orientation)
{
    mem.reset();

    int expanded=0;
    int generated=0;
    int h = compute_h(env, start, goal, ht, ns);

    
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
    std::vector<double> vertex_sens_decay_table(1, 1.0);

    s_node* goal_node = nullptr;
    int neighbors[4];
    int next_neighbors[4];



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

            cost = curr->g+1;
            tie_breaker = 0.0;

            assert(next >= 0 && next < env->map.size());
            depth = curr->depth + 1;

            //moving direction
            //flow
            op_flow = 0;
            all_vertex_flow = 0;

            h = compute_h(env, next, goal, ht, ns);

            diff = next - curr->id;
            d = get_d(diff,env);

            // Inject extra_weights direction penalty
            if (extra_weights != nullptr &&
                curr->id >= 0 && curr->id < static_cast<int>(extra_weights->size()) &&
                d >= 0 && d < static_cast<int>((*extra_weights)[curr->id].size())) {
                cost += (*extra_weights)[curr->id][d];
            }

            if (curr->parent != nullptr){
                p_diff = curr->id - curr->parent->id;
                p_d = get_d(p_diff,env);
                if (p_d != d) {
                    cost += std::max(0, orz_engine::cfg().flow_turn_penalty);
                    tie_breaker = std::max(0.0f, orz_engine::cfg().flow_turn_tie);
                }
                //tie breaking on prefering moving forward
            } else if (start_orientation >= 0) {
                const int turns = turn_steps(start_orientation, d);
                if (turns > 0) {
                    cost += turns * std::max(0, orz_engine::cfg().flow_start_turn_penalty);
                    tie_breaker = std::max(tie_breaker,
                                           turns * static_cast<double>(
                                               std::max(0.0f, orz_engine::cfg().flow_turn_tie)));
                }
            }


            temp_op = ( (unscale_flow(flow[curr->id].d[d])+1) * unscale_flow(flow[next].d[(d+2)%4]));

            //all vertex flow
            //the sum of all out going edge flow is the same as the total number of vertex visiting.
            temp_vertex = vertex_flow_count_at(flow, next);

            op_flow += temp_op;
        
            const VertexCostBreakdown vertex_cost =
                vertex_flow_cost(temp_vertex, depth - 1, vertex_sens_decay_table);
            all_vertex_flow += vertex_cost.cost;

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
                        std::cout << "error in astar: re-expansion" << std::endl;
                        assert(false);
                        exit(1);
                    }

                } 
            }
        }
            

          
    }


    if (goal_node == nullptr){
        std::cout << "error in astar: no path found "<< start<<","<<goal << std::endl;
        assert(false);
        exit(1);
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
