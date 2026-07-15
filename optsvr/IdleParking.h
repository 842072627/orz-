#pragma once

#include "SharedEnv.h"

#include <vector>

namespace optsvr {

class IdleParking {
public:
    void initialize(SharedEnvironment* env);
    void reset();

    bool enabled() const { return initialized_ && ratio_ > 0.0f && !parked_agents_.empty(); }
    bool is_parked(int agent_id) const;
    int parked_count() const { return static_cast<int>(parked_agents_.size()); }

    void apply_schedule_filter(std::vector<int>& schedule) const;
    void apply_goal_locations(SharedEnvironment* env) const;

private:
    std::vector<int> collect_outer_goals(const SharedEnvironment* env) const;

    bool initialized_ = false;
    int rows_ = 0;
    int cols_ = 0;
    int n_agents_ = 0;
    float ratio_ = 0.0f;

    std::vector<unsigned char> parked_;
    std::vector<int> parked_goals_;
    std::vector<int> parked_agents_;
};

IdleParking& idle_parking();

}  // namespace optsvr
