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

尚未实现：
- MQTT 接入
- ROS1 适配
- ROS2 适配
- SCAN 路线发布
- TaskManager 与 NavigationCoordinator 集成
- 暂停继续
- RouteManager
- 起步和终点控制
- 安全层
- 真实机器人速度输出

## 任务生命周期

```text
无活动任务
    ↓ START_TASK
活动任务锁定
    ├── UPDATE_MAX_VX：只更新速度
    ├── START_TASK：拒绝替换
    └── CANCEL_TASK：解除锁定
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
