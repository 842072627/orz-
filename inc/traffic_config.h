// traffic_config.h — Traffic-Aware Schedule 独立配置
#pragma once

// ====== 批次匹配 ======
#define BM_BATCH_SIZE           25      // 每批挑几个agent
#define BM_ALLOC_RATIO          0.60f   // 每批分配比例
#define BM_COL_SEGMENTS         8       // 按列分几段挑人
#define BM_MAX_ACTIVE_RATIO     0.60f   // 全局活跃率上限

// ====== Zone 范围过滤 ======
#define TRAFFIC_ZONE_STEP       1       // main_dist 分段粒度
#define TRAFFIC_ZONE_RANGE      100     // agent 只看 ±N 段内的任务

// ====== Traffic Scoring ======
#define FF_GLOBAL_W             0.5f    // 全局超载惩罚系数
#define FF_DIR_W                2.0f    // 方向失衡惩罚系数
#define FF_DECAY_K              0.01f   // future flow 衰减
