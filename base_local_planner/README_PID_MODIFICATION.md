# TrajectoryPlannerROS PID Rotation Control Modification

## 问题描述
原始的 `TrajectoryPlannerROS` 在机器狗到达目标位置（xy_goal）但需要调整朝向时，会出现持续转圈而无法停止的问题。这是由于原来的 `rotateToGoal` 函数使用的控制逻辑容易导致震荡。

## 解决方案
将原来的基于角度误差直接计算速度的方法改为基于PID控制器的方法，提供更稳定和精确的角度控制。

## 修改内容

### 1. 头文件修改 (`trajectory_planner_ros.h`)
添加了以下PID控制器相关的成员变量：
```cpp
// PID controller variables for rotation
double pid_kp_, pid_ki_, pid_kd_;           // PID gains
double pid_integral_error_;                 // Integral term accumulator
double pid_previous_error_;                 // Previous error for derivative
ros::Time pid_last_time_;                   // Last update time
double pid_max_integral_;                   // Anti-windup limit
```

### 2. 源文件修改 (`trajectory_planner_ros.cpp`)

#### 构造函数初始化
在两个构造函数中添加了PID参数的默认值初始化：
- `pid_kp_ = 2.0` (比例增益)
- `pid_ki_ = 0.0` (积分增益)  
- `pid_kd_ = 0.1` (微分增益)
- `pid_max_integral_ = 0.5` (积分限幅)

#### 参数加载
在 `initialize` 函数中添加了从ROS参数服务器读取PID参数：
```cpp
private_nh.param("rotation_pid_kp", pid_kp_, 2.0);
private_nh.param("rotation_pid_ki", pid_ki_, 0.0);
private_nh.param("rotation_pid_kd", pid_kd_, 0.1);
private_nh.param("rotation_pid_max_integral", pid_max_integral_, 0.5);
```

#### PID状态重置
在 `setPlan` 函数中添加了PID状态重置，确保每次新路径开始时PID控制器都是干净的状态。

#### `rotateToGoal` 函数重写
完全重写了 `rotateToGoal` 函数，实现了基于PID的角度控制：
1. **误差计算**: 使用 `angles::shortest_angular_distance` 计算最短角度误差
2. **时间管理**: 正确处理时间增量计算
3. **PID计算**: 实现标准PID算法（比例+积分+微分）
4. **积分防饱和**: 限制积分项避免积分饱和
5. **速度限制**: 应用最大/最小速度和加速度限制
6. **目标检测**: 当角度误差小于容忍度时自动停止并重置PID状态

## 参数调优指南

### PID参数说明
- **rotation_pid_kp** (比例增益): 控制对当前误差的响应强度
  - 数值越大响应越快，但可能导致超调
  - 推荐范围: 1.0 - 4.0
  
- **rotation_pid_ki** (积分增益): 消除稳态误差
  - 数值越大越能消除偏差，但可能引起震荡
  - 推荐范围: 0.0 - 0.5 (建议从0.0开始)
  
- **rotation_pid_kd** (微分增益): 减少超调和改善稳定性
  - 数值越大阻尼越强，但可能使响应变慢
  - 推荐范围: 0.05 - 0.3
  
- **rotation_pid_max_integral**: 积分项的最大值，防止积分饱和
  - 推荐范围: 0.3 - 1.0

### 调优步骤
1. 从推荐的默认值开始: Kp=2.0, Ki=0.0, Kd=0.1
2. 如果出现震荡，减小Kp或增大Kd
3. 如果存在稳态误差，缓慢增加Ki（从0.1开始）
4. 如果响应太慢，增加Kp
5. 对于四足机器人，考虑使用稍高的Kd值以获得更好的稳定性

## 使用方法

### 1. 配置参数
在你的ROS参数文件中添加PID参数：
```yaml
move_base:
  TrajectoryPlannerROS:
    rotation_pid_kp: 2.0
    rotation_pid_ki: 0.0
    rotation_pid_kd: 0.1
    rotation_pid_max_integral: 0.5
```

### 2. 监控调试
使用 `ROS_DEBUG` 日志查看PID控制器的工作状态：
```bash
rosservice call /move_base/set_logger_level "logger: 'ros.base_local_planner'" "level: 'debug'"
```

## 预期效果
- 消除机器狗在目标朝向调整时的持续转圈问题
- 提供更平滑、更精确的角度控制
- 减少震荡和超调现象
- 更快的收敛到目标朝向

## 文件清单
- `trajectory_planner_ros.h` - 添加了PID控制器成员变量
- `trajectory_planner_ros.cpp` - 实现了PID控制逻辑
- `trajectory_planner_pid_example.yaml` - 示例配置文件
- `README_PID_MODIFICATION.md` - 本说明文档
