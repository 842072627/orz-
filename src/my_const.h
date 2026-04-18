#ifndef MY_CONST_H
#define MY_CONST_H
namespace MyPlanner
{
    // pibt runtime (ms) per 100 agents. 
    const int PIBT_RUNTIME_PER_100_AGENTS = 1;

    //The minimal time left for PIBT for multiple steps in ms. 
    const int MIN_PIBT_TIME = 100; 

    // Traffic flow assignment end time tolerance in ms.
    const int TRAFFIC_FLOW_ASSIGNMENT_END_TIME_TOLERANCE = 10;

    // The default planner timelimit tolerance in ms.
    const int PLANNER_TIMELIMIT_TOLERANCE = 10;

    // The default scheduler timelimit tolerance in ms.
    const int SCHEDULER_TIMELIMIT_TOLERANCE = 10;

}
#endif
