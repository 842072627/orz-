// TaskScheduler.cpp - Strategy: efficiency (V13 best known: task aging + single-errand bonus)
#define STRATEGY_EFFICIENCY
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#if defined(STRATEGY_EFFICIENCY)
#include "strategy_efficiency.h"
void TaskScheduler::initialize(int pre) { strategy_init(); int lim = pre/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE; DefaultPlanner::schedule_initialize(lim, env); }
void TaskScheduler::plan(int tl, std::vector<int>& ps) { strategy_plan(tl, env, ps); }
#else
#error "Define STRATEGY_EFFICIENCY"
#endif
