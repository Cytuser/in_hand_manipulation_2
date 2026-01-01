# Low Level Hold 检查机制说明

## 📝 修改概述

为 `test_system_with_reset.py` 添加了与 High Level 类似的 **Hold 检查机制**，确保任务完成判定更加稳定可靠。

---

## 🔄 修改前后对比

### ❌ 修改前（单次检查）
```python
def check_task_completion(self):
    # ... 计算误差 ...
    is_completed = angle_error_deg < self.error_thres_deg  # 单次满足即完成
    return is_completed, angle_error_deg
```

**问题**：
- 物体可能只是"路过"目标位置
- 瞬时满足但不稳定也会判定完成
- 容易产生误判

---

### ✅ 修改后（Hold 机制）
```python
def check_task_completion(self):
    # ... 计算误差 ...
    
    # Hold 机制：需要连续满足阈值
    if angle_error_deg < self.error_thres_deg:
        self.success_counter += 1
    else:
        self.success_counter = 0  # 一旦超出阈值，重置计数器
    
    # 只有连续满足多次才判定完成
    is_completed = self.success_counter >= self.hold_steps_required
    
    return is_completed, angle_error_deg, self.success_counter
```

**优势**：
- ✅ 确保物体真正"停稳"在目标位置
- ✅ 提高任务完成判定的可靠性
- ✅ 减少误判，提高实验统计准确性

---

## 📊 详细修改内容

### 1. 添加参数（第50-56行）

```python
# parameters
self.timeout = 60.0
self.error_thres_deg = 5.0
self.hold_steps_required = 5  # 需要连续满足的次数（5秒 @ 1Hz）

# hold 检查相关
self.success_counter = 0       # 连续成功计数器
```

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `error_thres_deg` | 5.0° | 误差阈值 |
| `hold_steps_required` | 5 | 需要连续满足的次数 |
| `success_counter` | 0 | 当前连续成功计数 |

**时间计算**：
- 定时器频率：1Hz (每秒检查一次)
- Hold 时长：5 次 × 1秒 = **5秒**

---

### 2. 重置计数器（第172-187行）

```python
def reset_buffers(self):
    # ... 原有重置逻辑 ...
    
    # 重置 hold 计数器
    self.success_counter = 0
```

每次开始新任务时，计数器自动归零。

---

### 3. 更新检查逻辑（第292-321行）

```python
def check_task_completion(self):
    """
    检查是否达到目标SO3（带 hold 机制）
    
    Returns:
        is_completed (bool): 是否完成任务
        angle_error_deg (float): 当前角度误差（度）
        success_counter (int): 连续成功计数
    """
    # 归一化四元数
    q_curr = self.curr_quat / np.linalg.norm(self.curr_quat)
    q_target = self.curr_quat_target / np.linalg.norm(self.curr_quat_target)
    
    # 四元数点积（考虑双重覆盖，取绝对值）
    dot_product = np.abs(np.dot(q_curr, q_target))
    dot_product = np.clip(dot_product, 0.0, 1.0)
    
    # 计算角度误差（弧度→度）
    angle_error_rad = 2 * np.arccos(dot_product)
    angle_error_deg = np.rad2deg(angle_error_rad)
    
    # Hold 机制：需要连续满足阈值
    if angle_error_deg < self.error_thres_deg:
        self.success_counter += 1
    else:
        self.success_counter = 0  # 一旦超出阈值，重置计数器
    
    # 只有连续满足多次才判定完成
    is_completed = self.success_counter >= self.hold_steps_required
    
    return is_completed, angle_error_deg, self.success_counter
```

**返回值变化**：
- 原来：`(is_completed, angle_error)`
- 现在：`(is_completed, angle_error, success_counter)`

---

### 4. 增强日志输出（第338-358行）

```python
# 检查任务完成（带 hold 机制）
is_completed, angle_error, hold_count = self.check_task_completion()

# 打印当前状态（每次都显示，便于调试）
self.get_logger().info(
    f'[Check] Goal {self.current_goal_id + 1}: '
    f'Error={angle_error:.2f}° | Hold={hold_count}/{self.hold_steps_required} | '
    f'Time={elapsed_time:.1f}s'
)

if is_completed:
    # 任务成功完成（连续满足 hold 要求）
    self.success_count += 1
    self.completion_times.append(elapsed_time)
    self.current_goal_id += 1
    self.get_logger().info(
        f'✓ Goal {self.current_goal_id} COMPLETED! '
        f'Time: {elapsed_time:.2f}s, Final Error: {angle_error:.2f}°'
    )
```

**日志输出示例**：
```
[INFO] [Check] Goal 1: Error=4.23° | Hold=1/5 | Time=3.5s
[INFO] [Check] Goal 1: Error=3.87° | Hold=2/5 | Time=4.5s
[INFO] [Check] Goal 1: Error=2.91° | Hold=3/5 | Time=5.5s
[INFO] [Check] Goal 1: Error=2.45° | Hold=4/5 | Time=6.5s
[INFO] [Check] Goal 1: Error=2.18° | Hold=5/5 | Time=7.5s
[INFO] ✓ Goal 1 COMPLETED! Time: 7.50s, Final Error: 2.18°
```

---

## ⚙️ 参数调整指南

### 场景 1: 高精度任务
```python
self.error_thres_deg = 3.0            # 更严格的误差阈值
self.hold_steps_required = 10         # 更长的保持时间（10秒）
```

### 场景 2: 快速实验
```python
self.error_thres_deg = 8.0            # 更宽松的误差阈值
self.hold_steps_required = 3          # 更短的保持时间（3秒）
```

### 场景 3: 平衡模式（默认）
```python
self.error_thres_deg = 5.0            # 中等误差阈值
self.hold_steps_required = 5          # 中等保持时间（5秒）
```

---

## 🎯 与 High Level 的对比

| 特性 | **High Level** | **Low Level (修改后)** |
|------|----------------|------------------------|
| 误差阈值 | 3.0° | 5.0° |
| Hold 步数 | 10 步 | 5 步 |
| 检查频率 | ~10Hz (每个 MPC 迭代) | 1Hz (定时器) |
| Hold 时长 | 1 秒 (10步 × 0.1s) | 5 秒 (5步 × 1s) |
| 精度 | 更严格 | 中等 |
| 稳定性 | 高频检查，快速响应 | 低频检查，长时间验证 |

**设计理念**：
- **High Level**: 高频检查 + 短时间 hold（实时控制）
- **Low Level**: 低频检查 + 长时间 hold（批量实验）

两者互补，确保：
- High Level 快速反应
- Low Level 可靠验证

---

## 🔬 实验建议

### 1. 调试阶段
```python
self.hold_steps_required = 2  # 快速迭代，便于调试
```

### 2. 正式实验
```python
self.hold_steps_required = 5  # 确保结果可靠
```

### 3. 对比实验
可以通过修改参数来对比不同 hold 策略的效果：
- 无 hold (`hold_steps_required = 1`)
- 短 hold (`hold_steps_required = 3`)
- 长 hold (`hold_steps_required = 10`)

---

## 📈 预期效果

### 成功率提升
- **修改前**：可能有 5-10% 的误判（瞬时到达但未稳定）
- **修改后**：误判率接近 0%

### 完成时间延长
- **修改前**：平均完成时间 T 秒
- **修改后**：平均完成时间 T + 5 秒（增加 hold 验证时间）

**权衡**：
- ✅ 提高可靠性
- ⚠️ 略微增加完成时间
- 💡 整体实验质量提升

---

## 🐛 故障排查

### 问题 1: 任务永远无法完成
**可能原因**：
- 误差阈值太严格
- Hold 步数太多
- 控制器性能不足

**解决方案**：
1. 增大 `error_thres_deg`（如 5° → 8°）
2. 减少 `hold_steps_required`（如 5 → 3）
3. 检查控制器日志，确认是否收敛

### 问题 2: 任务完成太慢
**可能原因**：
- Hold 步数过多

**解决方案**：
- 减少 `hold_steps_required`（如 5 → 3）

### 问题 3: 日志输出过多
**可能原因**：
- 每秒都打印检查信息

**解决方案**：
```python
# 只在接近完成时打印
if hold_count > 0 or is_completed:
    self.get_logger().info(...)
```

---

## 📚 相关文件

- **当前文件**: `test_system_with_reset.py`
- **High Level**: `high_level/planner/ddp/tasks/inhand_ddp_full_hand_quat.py`
  - `SuccessChecker` 类（第220-258行）
- **文档**: `high_level/planner/ddp/tasks/TASK_COMPLETION_USAGE.md`

---

## ✅ 测试检查清单

运行实验前，确认：
- [ ] 参数设置合理（根据任务精度要求）
- [ ] 日志输出正常（能看到 Hold 计数）
- [ ] 计数器正确重置（每个新任务从 0 开始）
- [ ] 超时机制正常（不会无限等待）

---

## 🎉 总结

通过添加 Hold 检查机制：
1. ✅ 提高任务完成判定的**可靠性**
2. ✅ 确保物体真正**停稳**在目标位置
3. ✅ 减少实验统计中的**误判**
4. ✅ 与 High Level 保持**一致性**

**建议**：在正式实验中始终启用 Hold 机制，以获得最可靠的结果！


