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
- 独立 RouteCorridorObservationGate
- SCAN 三维膨胀占据评估结果验证
- 任务 sequence 关联校验
- 地图时间戳超时/未来检查
- 路线进度滞后/未来检查
- 地图外采样结果处理
- 14 种评估结果状态
- navdog_scan_adapter 三维采样评估器
- GridMap::getInflateOccupancy(pos, yaw) 查询
- 三维 x/y/z + yaw 膨胀占据语义
- 采样间隔 = 分辨率 × 0.5
- 最早阻塞距离
- 最早阻塞路线累计位置
- 已走路线障碍物忽略
- 单点路线走廊检查
- NavigationModeManager 导航子模式管理
- ROUTE_FOLLOW / LOCAL_AVOID / ROUTE_REJOIN 三态状态机
- 路线阻塞确认时间（0.2s）
- 立即绕障触发（≤0.8m）
- LOCAL_AVOID 最短保持时间（0.5s）
- 路线恢复确认时间（0.4s）
- 接回路线横向+航向确认时间（0.3s）
- 多轮绕障切换
- TaskMode 差异策略（NORMAL_AVOID / ROUTE_ONLY / CHARGING）
- 模式诊断输出（NavigationModeStatus）
- 浮点时间 epsilon 防边界误判

尚未实现：
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
    ├── 路线进度有效后调用 RouteCorridorObservationGate
    ├── 调用 NavigationModeManager 决定导航子模式
    ├── CLEAR / BLOCKED → 输出 TRACKING_STOP + route_progress + route_corridor + navigation_mode
    ├── 观察缺失/过期/非法 → 输出 TRACKING_STOP（navigation_mode 保持当前模式）
    └── INVALID_TIME / INVALID_CONFIG / INVALID_PROGRESS → FAILED

NavigationModeManager 只回答：
"当前应该使用哪种导航参考意图？"

它不负责：
"应该输出什么速度？"
"应该向左绕还是向右绕？"
"具体接回哪个路点？"

本阶段只输出模式意图，不输出真实导航速度。
所有 TRACKING 输出仍然为 TRACKING_STOP（vx=0, vy=0, yaw_rate=0）。

## 导航子模式状态机

```text
CoreState::TRACKING
        |
        v
ROUTE_FOLLOW
   | BLOCKED <= 0.8m
   | 或 <=1.5m 持续0.2s
   v
LOCAL_AVOID
   | 最少0.5s
   | 路线CLEAR持续0.4s
   v
ROUTE_REJOIN
   | 横向<=0.2m
   | 航向<=12deg
   | 持续0.3s
   v
ROUTE_FOLLOW
```

ROUTE_REJOIN 途中重新 BLOCKED
→ 返回 LOCAL_AVOID（avoidance_cycle_count +1）

普通阻塞确认必须连续成立。
ROUTE_REJOIN 中任何 CLEAR 帧都会清除之前的 blocked confirmation。

确认时间配置为 0 时，
条件首次满足即可在同一 update 完成切换。

ROUTE_ONLY
→ 不进入 LOCAL_AVOID
→ 始终保持 ROUTE_FOLLOW
→ 近距离阻塞时 reason = ROUTE_ONLY_BLOCKED

CHARGING
→ 当前阶段允许 LOCAL_AVOID
→ 与 NORMAL_AVOID 行为一致
→ 不实现充电桩附近特殊行为

Corridor 暂时不可用（WAITING_FOR_OBSERVATION / STALE_MAP 等）
→ 保持当前模式
→ 清除所有连续确认计时器
→ route_blocked 和 route_blocked_near 置 false
→ 不清除当前任务和绕障轮次
→ 不进入 FAILED

Corridor 内部错误（INVALID_TIME / INVALID_CONFIG / INVALID_PROGRESS）
→ INVALID_CORRIDOR_RESULT
→ Coordinator 转入 FAILED

新任务（sequence 变化）
→ 重新初始化为 ROUTE_FOLLOW
→ 清空所有计时器和 rejoin anchor
→ avoidance_cycle_count 归零

同一任务 UPDATE_MAX_VX
→ 不重置模式
→ 不清空计时器

RouteProgressTracker 只回答：
"机器人沿原始路线走到哪里了？"

RouteCorridorObservationGate 只回答：
"SCAN 三维评估结果是否可用于路线阻塞判断？"

ScanRouteCorridorEvaluator3D（navdog_scan_adapter）只回答：
"沿当前路线进度之后的前方路线，三维膨胀占据是否被阻塞？"

它们不负责：
"什么时候正式进入绕障？"
"应该向左绕还是向右绕？"
"什么时候重新接回路线？"
"应该输出什么速度？"

SCAN 使用三维雷达点云和三维膨胀栅格地图。

navdog 不再使用二维 ObstacleCircle 判断路线阻塞。

正式路线碰撞来源为：
GridMap::getInflateOccupancy(pos, yaw)

路线参考仍以地面 XY 为主，
但占据查询保留完整 x/y/z 和 yaw 语义。

当前查询高度使用 RobotState::z。

当前阶段是三维地图碰撞查询，
但地面路线采样高度使用实时 RobotState::z。
未来只有在 RoutePoint 增加明确 has_z 语义后，
才允许切换为路线 z 插值。

/grid_map/occupancy_inflate 是可视化输出，
不是 navdog 正式碰撞判断的数据来源。

navdog_scan_adapter 不可自行增加水平半径、
z 膨胀或双圆柱偏移；这些已由 SCAN GridMap 内部处理。

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
