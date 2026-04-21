# A11方向：轻量级路径修复

## 问题分析

当前架构问题：
- frank_wolfe在plan开始时固定guide path
- DEV-336通过deviation>5触发重计算，但只在step 0后有效
- multi-step循环中（step>0）guide path不会更新

## 方案：每步PIBT后运行轻量级neighbor修复

在`run_multistep_pibt_once`后：
1. 检测哪些agent的路径在下一步会有冲突
2. 对这些agent运行轻量级路径重规划（只重规划1-2步）
3. 不运行完整的frank_wolfe，只做局部修复

## 实现思路

```cpp
// 在run_multistep_pibt_once后添加
if (step > 0 && has_conflicts()) {
    // 对冲突的neighborhood进行局部重规划
    lightweight_repair(neighborhood);
}
```

## 预期

- 计算开销小（只重规划neighborhood）
- 可能在保持性能的同时减少冲突
- 不会显著增加总时间

## 验证

运行run_mini_md5.ps1测试，确认：
1. fulfill不下降（或轻微下降）
2. total提升（如果有）
