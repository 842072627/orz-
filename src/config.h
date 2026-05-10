// config.h â€” Combined Track å…¨å±€é…ç½®
// æ‰€æœ‰åœ°å›¾ç­–ç•¥é€‰æ‹©åœ¨æ­¤é›†ä¸­æŽ§åˆ¶

#pragma once
#include <string>
#include "SharedEnv.h"

// ========== åœ°å›¾è¯†åˆ« ==========
inline bool is_ORZ(SharedEnvironment* env) {
    return env->map_name.find("orz") != std::string::npos;
}
inline bool is_BOS(SharedEnvironment* env) {
    return env->map_name.find("bos") != std::string::npos ||
           env->map_name.find("Boston") != std::string::npos;
}
inline bool is_FULFILL(SharedEnvironment* env) {
    return env->map_name.find("fulfill") != std::string::npos;
}
inline bool has_custom_strategy(SharedEnvironment* env) {
    return is_ORZ(env);  // åªæœ‰ORZæœ‰å®šåˆ¶ç­–ç•¥
}

// ========== ç­–ç•¥é€‰æ‹© ==========
#define ORZ_SCHEDULER   "centerline"    // ORZè°ƒåº¦å™¨: centerline=ä¸­å¿ƒçº¿çƒ­åŠ›å›¾+è´ªå¿ƒåŒ¹é… (v2)
#define ORZ_PLANNER     "default"       // ORZè§„åˆ’å™¨: é»˜è®¤PIBTï¼ˆå¾…å®šåˆ¶ï¼‰
#define ORZ_EXECUTOR    "default"       // ORZæ‰§è¡Œå™¨: é»˜è®¤ï¼ˆå¾…å®šåˆ¶ï¼‰

// ========== åŠŸèƒ½å¼€å…³ ==========
#define ORZ_PIBT_DIRECTION_BLOCK  true    // è°ƒåº¦å™¨+PIBTæ–¹å‘é˜»å¡žè”åŠ¨
#define ORZ_DIRECTION_LOCK        true    // æ–¹å‘é”ï¼ˆlock_dirï¼‰ï¼Œå…³é—­åŽåªä¿ç•™HOT_BLOCKæ‹¥å¡žå°é”

// è¿è¡Œæ—¶èŽ·å–ç­–ç•¥åï¼ˆç”¨äºŽTaskScheduler.cppï¼‰
inline std::string get_orz_strategy() { return ORZ_SCHEDULER; }



