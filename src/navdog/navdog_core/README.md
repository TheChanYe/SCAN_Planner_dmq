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

当前只建立安全零输出骨架，没有实现真实导航。

所有 `update()` 输出均为安全零速度（`vx=0, vy=0, yaw_rate=0`），状态固定为 `IDLE`。

## 构建

```bash
cd <workspace>
catkin_make
```

运行测试：

```bash
catkin_make run_tests_navdog_core
```
