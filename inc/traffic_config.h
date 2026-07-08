// traffic_config.h — Traffic-Aware Schedule 独立配置
#pragma once

// ====== 批次匹配 ======
#define BM_BATCH_SIZE           15      // 每批挑几个agent (10-20)
#define BM_ALLOC_RATIO          0.80f    // 每批分配比例 (80%)
#define BM_COL_SEGMENTS         8       // 按列分几段挑人（确保不同走廊）

// ====== Traffic Scoring ======
#define FF_GLOBAL_W             2.0f    // 全局超载惩罚系数
#define FF_DIR_W                1.0f    // 方向失衡惩罚系数
#define FF_DIR_RATIO            1.5f    // fr > fl*1.5 认为方向偏了
#define FF_DECAY_K              0.01f   // future flow 衰减 (1/(1+cumul*K))
