// cold_zone_pushout.h — DEPRECATED (replaced by control_zone.h + narrow_passage.h)
// Kept only for backward compatibility during transition.
#pragma once
#include "SharedEnv.h"
#include "centerline.h"
#include "config.h"

namespace ColdZonePushout {
inline void init(const SharedEnvironment* env) { (void)env; }
inline void update(SharedEnvironment* env) { (void)env; }
}
