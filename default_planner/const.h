
#ifndef CONST_H
#define CONST_H
namespace DefaultPlanner
{
    // pibt runtime (ms) per 100 agents. 
    // ARCH-J: Increased from 1 to 5 for large-scale cloud maps (10000 agents)
    const int PIBT_RUNTIME_PER_100_AGENTS = 5;

    // Traffic flow assignment end time tolerance in ms.
    // The default planner will end the traffic flow assignment phase this many milliseconds before traffic flow assignment end time.
    const int TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE = 10;


    // The default planner timelimit tolerance in ms.
    // ARCH-J: Reduced from 10 to 5 for more planning time on large maps
    const int PLANNER_TIMELIMIT_TOLERANCE = 5;

    // The default scheduler timelimit tolerance in ms.
    const int SCHEDULER_TIMELIMIT_TOLERANCE = 5;



}
#endif