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

尚未实现：
- MQTT 接入
- ROS1 适配
- ROS2 适配
- SCAN 路线发布
- 暂停继续
- RouteManager
- 起步和终点控制
- 安全层
- 真实机器人速度输出
- 真实 SCAN PlannerFeedback ROS 适配
- START_ALIGN 实际旋转控制
- START_ALIGN → TRACKING
- TRACKING 阶段 planner_cmd

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
