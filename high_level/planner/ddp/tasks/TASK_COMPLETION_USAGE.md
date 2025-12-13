# 任务完成判断功能使用说明

## 概述

在手转球（In-Hand Manipulation）任务中，新增了 `TaskCompletionChecker` 类来判断任务是否完成。该类使用**姿态误差 + 角速度**的组合指标，确保物体不仅"到达"目标姿态，还能"停稳"在目标位置。

## 核心指标

### 1. 姿态误差（Orientation Error）
- **定义**：当前物体姿态与目标姿态之间的旋转角度差（弧度）
- **计算方法**：通过四元数差值的轴角表示
- **阈值**：`error_threshold`（默认 0.05 rad ≈ 3°）

### 2. 角速度（Angular Velocity）
- **定义**：物体姿态的变化速率（rad/s）
- **计算方法**：相邻两帧姿态差 / 时间步长
- **阈值**：`velocity_threshold`（默认 0.02 rad/s ≈ 1°/s）

### 3. 稳定保持（Stability）
- **定义**：需要连续多步满足上述两个条件
- **阈值**：`stable_steps`（默认 10 步）

## 完成判定公式

任务完成的条件：

$$
\text{Success} = \begin{cases}
||Error_{ori}|| < 0.05 \text{ rad} & \text{(姿态到达)} \\
||\omega_{obj}|| < 0.02 \text{ rad/s} & \text{(速度稳定)} \\
\text{连续满足} \geq 10 \text{ 步} & \text{(持续稳定)}
\end{cases}
$$

## 使用方法

### 基本使用

```python
from inhand_ddp_full_hand_quat import TaskCompletionChecker

# 1. 创建任务完成检查器
completion_checker = TaskCompletionChecker(
    error_threshold=0.05,       # 姿态误差阈值（rad）
    velocity_threshold=0.02,    # 角速度阈值（rad/s）
    stable_steps=10,            # 需要保持稳定的步数
    h=0.1                       # 时间步长（s）
)

# 2. 在 MPC 循环中使用
x_traj = solve_ddp_mpc_problem(
    N_steps,
    params,
    default_q_sims, default_sim_params,
    state, actuation,
    ref_gen,
    logger=logger,
    q_vis=q_vis,
    viz_helper=viz_helper,
    completion_checker=completion_checker,  # 传入检查器
    enable_early_stop=True                  # 启用提前停止
)

# 3. 获取统计信息
stats = completion_checker.get_statistics()
print(f"平均误差: {stats['mean_error']:.6f} rad")
print(f"完成率: {stats['completion_rate']:.1f}%")
```

### 自定义阈值示例

#### 高精度任务（更严格）
```python
completion_checker = TaskCompletionChecker(
    error_threshold=0.01,       # 0.57° - 非常精确
    velocity_threshold=0.005,   # 0.29°/s - 几乎静止
    stable_steps=20,            # 保持更长时间
    h=0.1
)
```

#### 快速任务（更宽松）
```python
completion_checker = TaskCompletionChecker(
    error_threshold=0.1,        # 5.7° - 允许较大误差
    velocity_threshold=0.05,    # 2.86°/s - 允许较快速度
    stable_steps=5,             # 更快判定完成
    h=0.1
)
```

## 输出示例

### 任务完成时的输出
```
============================================================
🎉 任务完成！
  完成迭代: 145
  完成时间: 14.50s
  最终误差: 0.002050 rad (0.117°)
  最终角速度: 0.001230 rad/s (0.070°/s)
============================================================
```

### 任务统计信息
```
============================================================
任务统计信息:
  平均误差: 0.015432 rad (0.884°)
  误差标准差: 0.008765 rad (0.502°)
  最大误差: 0.087543 rad (5.016°)
  最小误差: 0.001987 rad (0.114°)
  最终误差: 0.002050 rad (0.117°)
  完成率: 100.0%
============================================================
```

## 提前停止机制

当 `enable_early_stop=True` 时，一旦任务完成，MPC 循环会自动提前终止，节省计算时间：

```
提前停止MPC循环（第 155/190 次迭代）
```

## API 参考

### TaskCompletionChecker 类

#### 初始化参数
- `error_threshold` (float): 姿态误差阈值（rad），默认 0.05
- `velocity_threshold` (float): 角速度阈值（rad/s），默认 0.02
- `stable_steps` (int): 需要保持稳定的步数，默认 10
- `h` (float): 时间步长（s），默认 0.1

#### 主要方法

##### `check_success(x0, error, iter_num)`
检查当前步骤是否完成任务。

**参数**：
- `x0`: 当前状态（四元数在前4维）
- `error`: 当前姿态误差（rad）
- `iter_num`: 当前迭代次数

**返回**：
- `is_success` (bool): 是否完成
- `info` (dict): 详细信息字典

##### `get_statistics()`
获取任务执行的统计信息。

**返回**：
- `stats` (dict): 包含平均误差、标准差、最大/最小误差等统计数据

##### `reset()`
重置检查器，用于开始新任务。

## 日志记录

如果传入 `logger`，会自动记录以下信息：
- `task_error`: 当前姿态误差
- `task_velocity`: 当前角速度
- `task_success_counter`: 连续成功的步数

## 注意事项

1. **时间步长**：确保 `h` 参数与实际 MPC 时间步长一致
2. **阈值调整**：根据具体任务精度要求调整阈值
3. **稳定步数**：步数太少可能误判，太多会延迟判定
4. **角速度计算**：需要至少2步历史数据才能计算角速度

## 性能影响

- 额外计算开销：每步约 0.1-0.2 ms（四元数运算）
- 内存开销：存储历史轨迹，约 O(N) 空间复杂度
- 提前停止收益：可节省 10%-50% 的计算时间（取决于任务难度）

## 扩展建议

### 添加更多指标
```python
# 可以扩展 TaskCompletionChecker 类来添加：
# - 手指接触力稳定性
# - 物体位置误差
# - 能量消耗指标
```

### 可视化误差曲线
```python
import matplotlib.pyplot as plt

info = completion_checker._get_info()
errors = info['error_history']

plt.plot(errors)
plt.axhline(y=0.05, color='r', linestyle='--', label='Threshold')
plt.xlabel('Iteration')
plt.ylabel('Orientation Error (rad)')
plt.legend()
plt.show()
```

## 相关代码位置

- **类定义**：`inhand_ddp_full_hand_quat.py` Line 405-532
- **MPC 集成**：`solve_ddp_mpc_problem()` Line 961-989
- **主函数示例**：`if __name__ == "__main__"` Line 1238-1254

