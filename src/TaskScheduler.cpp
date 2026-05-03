// TaskScheduler.cpp - Strategy: heatmap (congestion-aware)
#define STRATEGY_HEAT
#include "TaskScheduler.h"
#include "scheduler.h"
#include "const.h"
#if defined(STRATEGY_HEAT)
#include "strategy_heat.h"
void TaskScheduler::initialize(int pre) { strategy_heat::init(); int lim = pre/2 - DefaultPlanner::SCHEDULER_TIMELIMIT_TOLERANCE; DefaultPlanner::schedule_initialize(lim, env); }
void TaskScheduler::plan(int tl, std::vector<int>& ps) { strategy_heat::plan(tl, env, ps); }
#else
#error "Define strategy"
#endif
