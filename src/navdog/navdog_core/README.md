# navdog_core

## 包职责

navdog_core 负责：

- 导航业务状态
- 通用数据结构
- 最终控制协调
- 与 ROS 无关的导航决策

## 非职责

navdog_core 不负责：

- MQTT 通信
- ROS topic
- ROS 参数加载
- SCAN 三维地图
- B 样条规划
- 传感器消息解析
- 机器人底盘发布

## 依赖方向

```text
navdog_ros1  ─┐
navdog_ros2  ─┼──> navdog_core
navdog_mqtt ──┘

navdog_core 不反向依赖任何适配层。
```

## ROS2 迁移要求

navdog_core 保持不变，只新增或替换 navdog_ros2。

## 当前状态

当前已经实现：
- 通用导航数据结构
- 安全零输出 NavigationCoordinator 骨架
- 独立 TaskManager
- 单活动任务锁
- 内部任务 sequence 分配
- 任务取消
- 活动任务 max_vx 更新
- 任务和速度输入校验
- TaskManager 与 NavigationCoordinator 集成
- START_TASK 驱动 IDLE → PLANNING
- CANCEL_TASK 驱动 PLANNING → IDLE
- 一次性 PlannerAction 队列
- CANCEL 清除尚未执行的旧规划动作
- PLANNING 阶段安全零速度输出
- 抽象 PlannerFeedback 状态处理
- PlannerFeedback 与活动任务 sequence 关联校验
- 旧规划反馈过滤
- PLANNING → START_ALIGN
- PLANNING → FAILED
- 规划超时保护
- 规划失败后保持任务锁
- 独立 StartAlignController
- 路线起始方向解析
- 角度误差最短路径归一化
- START_ALIGN 进入/退出迟滞
- 原地旋转角速度控制
- START_ALIGN → TRACKING
- START_ALIGN 超时保护
- 非有限 SET_ROUTE 时间保护
- 独立 RouteProgressTracker
- 路线段投影
- 原始路线累计长度
- 剩余路线距离
- 横向路线偏差
- 路线当前方向
- 单调不回退的路线进度
- 受限前向搜索
- 重复路点和单点路线支持
- 独立 RouteCorridorEvaluator
- 世界坐标系二维障碍物输入
- 原始路线前向走廊检查
- 路线走廊半径模型
- 障碍物有效半径/膨胀半径支持
- 最早阻塞距离
- 最早阻塞路线累计位置
- 最小有符号安全间隙
- 障碍物时间戳和超时检查
- 已走路线障碍物忽略
- 单点路线走廊检查

尚未实现：
- NavigationModeManager
- ROUTE_FOLLOW / LOCAL_AVOID / ROUTE_REJOIN
- 绕障触发迟滞
- SCAN 局部绕障轨迹
- TRACKING planner_cmd 输出
- 障碍物安全层
- 终点控制
- ROS1/ROS2 适配
- MQTT 接入

注意：

PlannerFeedback.trajectory_id 必须等于活动 NavigationTask.sequence。
PlannerFeedback.stamp_sec 必须与 update(now_sec) 使用同一时间基准。

## 任务生命周期

```text
无活动任务
    ↓ START_TASK
活动任务锁定
    ├── UPDATE_MAX_VX：只更新速度
    ├── START_TASK：拒绝替换
    └── CANCEL_TASK：解除锁定
```

## 核心流程

```text
START_TASK
    ↓
TaskManager 接收并锁定任务
    ↓
NavigationCoordinator 进入 PLANNING
    ↓
update() 输出一次 SET_ROUTE
    ↓
等待规划适配层反馈

PLANNING
    ├── READY / EXECUTING → START_ALIGN
    ├── FAILED           → FAILED
    ├── 超时             → FAILED
    └── 无效或旧反馈      → 忽略

START_ALIGN
    ├── 初始误差 ≤ enter_deg → TRACKING
    ├── 开始旋转后误差 ≤ exit_deg → TRACKING
    ├── 超时 → FAILED
    └── 无有效方向 → FAILED

TRACKING
    ├── 调用 RouteProgressTracker 计算路线进度
    ├── 路线进度有效后调用 RouteCorridorEvaluator
    ├── CLEAR / BLOCKED → 输出 TRACKING_STOP + route_progress + route_corridor
    ├── 障碍物缺失/过期/非法 → 输出 TRACKING_STOP（route_corridor 无效）
    └── INVALID_* → FAILED

RouteProgressTracker 只回答：
“机器人沿原始路线走到哪里了？”

RouteCorridorEvaluator 只回答：
“当前原始路线前方是否被障碍物有效范围占用？”

它不负责：
“什么时候正式进入绕障？”
“应该向左绕还是向右绕？”
“什么时候重新接回路线？”
“应该输出什么速度？”

ObstacleCircle::effective_radius_m 已包含适配层赋予的有效半径。
不要在适配层和核心层重复增加同一膨胀距离。

注意：
START_ALIGN 仅输出 yaw_rate。
vx 和 vy 始终为 0。
TRACKING 当前仍然输出 TRACKING_STOP。
无论路线 CLEAR 还是 BLOCKED，TRACKING 都不输出非零速度。

CANCEL_TASK
    ↓
清除活动任务和旧规划动作
    ↓
NavigationCoordinator 回到 IDLE
    ↓
update() 输出一次 CANCEL
```

## 构建

```bash
cd <workspace>
catkin_make
```

运行测试：

```bash
catkin_make run_tests_navdog_core
```
