# 导盲犬导航项目开发要求

## 1. 文档用途

本文是本仓库的长期开发约束，适用于人工开发和 AI 辅助修改。开始分析或改代码前必须先阅读本文，并同时参考：

- `docs/ARCHITECTURE_BOUNDARIES.md`
- `docs/CODE_MAP.md`
- `docs/LOGGING_GUIDE.md`
- `docs/NAVIGATION_WORKFLOW.md`

需求与本文冲突时，先说明冲突、风险和建议方案，未经确认不得破坏既有分层。

## 2. 项目目标与运行环境

- 系统运行于 ROS Noetic，真实设备为 Linux arm64/NVIDIA 平台。
- 上层通过 MQTT 下发路线和任务控制，真实狗通过 MQTT 接收最终速度控制。
- `/rslidar_points` 提供 `sensor_msgs/PointCloud2` 三维雷达数据，SCAN 负责局部规划和三维避障。
- 里程计、雷达、MQTT 地址、MQTT/ROS 话题、狗体尺寸、雷达外参和速度限制必须可在真实狗 YAML 中配置。
- `src/simulator/` 仅用于仿真；真实狗驱动、配置和启动入口放在 `src/dmq_dog/`。
- 真实狗总启动入口为 `roslaunch dmq_dog bringup_real.launch`。新增真实运行节点时，应接入该总启动文件，不再要求用户分别启动多个必要节点。

## 3. 目录与职责

### `src/simulator/`

只放仿真环境、仿真传感器、地图生成和仿真机器人相关代码。真实狗接口、MQTT 或真实设备特例不得放入这里。

### `src/dmq_dog/`

真实导盲犬的设备集成 ROS 包：

- `config/robot_real.yaml`：真实狗统一运行配置。
- `launch/bringup_real.launch`：真实系统总启动入口。
- `launch/`：其他真实狗分启动和 RViz 启动文件。
- `rviz/`：真实系统 RViz 配置。
- `include/dmq_dog/`、`src/`：MQTT 客户端和真实设备 ROS 桥接代码。

该包负责设备接入和消息转换，不拥有导航状态机、避障算法或任务决策。

### `src/navdog/navdog_task/`

标准 C++ 任务层。只负责任务数据、任务会话、sequence 和合法任务转换，不依赖 ROS，不解析 MQTT，不控制机器人。

### `src/navdog/navdog_protocol/`

标准 C++ 协议层。负责 Mosquitto/JSON 与结构化 `NavigationEvent` 的转换，可依赖 Mosquitto/JsonCpp，但不得依赖 ROS，也不得决定导航模式、到达条件或速度。

### `src/navdog/navdog_core/`

标准 C++ 导航核心。负责路线进度、起步对齐、Route/SCAN 模式切换、目标到达、恢复和安全规则。不得包含 ROS API、MQTT、GridMap 或设备代码，也不得直接做日志 I/O；诊断信息通过结构化结果交给 Runtime。

### `src/navdog/navdog_scan_adapter/`

SCAN/GridMap 到 Core 数据结构的适配层。可依赖 ROS 和 GridMap，只负责生成障碍摘要、路线走廊评估和局部规划观察结果，不拥有导航状态机。

### `src/navdog/navdog_runtime/`

ROS 组合根。负责订阅/发布、定时循环、参数加载、调用 Core、记录 ROS 运行日志以及组织各适配层。不得复制 `NavigationModeManager`、到达判断或任务状态转换条件。

`cmd_vel_owner_mux` 是唯一允许发布 `/cmd_vel` 的节点。

### `src/planner/`

SCAN 原生规划代码。通用算法保留在原有包中，真实狗集成使用 `plan_manage_dmq`：

- SCAN 只负责地图、规划、轨迹和跟踪。
- `closed_loop_controller` 只发布 `/native_scan/raw_cmd`；Runtime将其交给Core的
  `SafetySupervisor`后再发布 `/navdog/scan_cmd`。
- SCAN 不解析 MQTT，不修改 Navdog 任务状态，不直接发布 `/cmd_vel`。
- 修改上游通用算法前必须确认不能在 `plan_manage_dmq` 或 adapter 中解决，避免污染原始模块。

## 4. ROS 与标准 C++ 分离

依赖方向必须保持：

```text
navdog_task <- navdog_core <- navdog_runtime
Mosquitto/JSON -> navdog_protocol -> navdog_runtime -> navdog_core
SCAN/GridMap -> navdog_scan_adapter -> navdog_runtime -> navdog_core
```

具体要求：

- `navdog_task`、`navdog_core` 不得包含 ROS 头文件、ROS 消息和 ROS 参数读取。
- `navdog_protocol` 不得包含 ROS 依赖。
- 纯 C++ 层使用普通数据结构、枚举和返回值表达状态，不通过 ROS 全局状态通信。
- ROS 消息转换、ROS 参数、时间戳和发布订阅只放在 Runtime、Adapter 或设备桥接层。
- `.hpp` 放公开类型和接口，`.cpp` 放实现；除简单值类型外，不在头文件堆积实现。
- 不跨包复制类型、阈值或状态判断；共享业务规则应只有一个所有者。

## 5. 配置要求

真实设备参数统一从 `src/dmq_dog/config/robot_real.yaml` 加载。至少包括：

- MQTT broker 地址、端口、client ID 和 MQTT 话题。
- 雷达点云、里程计、命令、状态及调试 ROS 话题。
- 狗体长、宽、高。
- 雷达 `xyz/rpy` 外参；默认位于狗体正前方往后 10 cm 的位置。
- Route 跟随速度、Local Avoid 避障速度、角速度和最终设备限幅。
- 地图、路线走廊、安全距离、模式切换、终点判定和恢复参数。

禁止在代码或 launch 中重复硬编码 YAML 已有参数。新增参数时必须：

1. 放到职责对应的 YAML 分组。
2. 给出单位明确的名称，如 `_m`、`_mps`、`_sec`、`_deg`。
3. 提供保守默认值和必要注释。
4. 在唯一配置加载位置读取并校验。
5. 为配置加载或边界行为补充对应测试。

仿真配置与真实配置分开，不能为了真实狗修改而改变仿真默认行为。

## 6. 控制链路与状态规则

控制链路必须保持单一所有权：

```text
ROUTE_FOLLOW -> /navdog/route_cmd
LOCAL_AVOID  -> /native_scan/raw_cmd -> Runtime/Core SafetySupervisor -> /navdog/scan_cmd
cmd_vel_owner_mux -> /cmd_vel
dmq_bridge -> MQTT physical command
```

- 任何节点不得绕过 mux 直接发布 `/cmd_vel`。
- 同一时刻只能有一个速度所有者；切换到 SCAN 时必须等待新鲜且属于当前任务的 SCAN 命令。
- Route 和 SCAN 的速度限制应分别配置，但最终输出还必须经过真实设备硬限幅。
- 规划器使用的运动模型必须与真实狗执行的命令一致。不得在碰撞检查后随意把 `vy` 转换成不同的 `yaw_rate`，除非规划和安全检查同时使用该真实运动模型。
- 最终安全保护必须针对实际发送给狗的命令生效，不能因进入 LOCAL_AVOID 而旁路。
- `ctrl=0` 表示停止、取消当前导航并解锁下一任务；相关活动标志、等待标志、计时器、SCAN 轨迹和命令所有权必须清理。
- 新路线只能在任务协议允许时接收；拒绝时必须记录明确原因，不能静默丢弃。
- `SUCCEEDED` 只能在满足最终位置和方向要求后产生。终点附近受阻、恢复超时或规划失败不能伪装成成功，应进入明确的受阻、失败或等待恢复状态。
- 里程计超时、跳变或不可能速度必须触发安全停车，定位恢复稳定前不得继续规划。

## 7. 日志要求

遵循 `docs/LOGGING_GUIDE.md`，并保持日志与所在层一致：

- ROS 节点使用 `ROS_INFO/WARN/ERROR` 及对应 throttle 宏。
- MQTT 协议和连接日志使用现有 MQTT logger 格式。
- SCAN 使用其现有 ROS/规划日志体系。
- 纯 C++ Core/Task 不直接打印日志；返回结构化状态，由 Runtime 记录 ROS 日志。
- 不得使用临时 `std::cout`、无节流高频日志或同一事件多层重复打印。
- 状态变化使用 INFO；异常、拒绝、恢复失败使用 WARN/ERROR；高频等待和传感器异常必须节流。
- 关键日志应包含任务 sequence、状态/模式、命令所有者、原因和必要的数值上下文。
- 排查完成后删除无价值的临时日志，只保留能够长期定位状态转换、任务生命周期和安全问题的日志。

日志职责固定：Runtime 记录 `NAV_STATE/NAV_MODE`，mux 记录 `CMD_OWNER`，MQTT 记录连接及任务接受/拒绝，SCAN 记录规划、碰撞和轨迹事件。

## 8. AI 修改代码时的工作要求

### 修改前

1. 先读取本文和相关架构文档。
2. 查看当前 `git status`，保留用户已有修改，不得覆盖或回退无关内容。
3. 先从日志、配置和调用链确认根因，不凭单条现象直接改参数或状态机。
4. 用户要求“先分析”时只能分析和给方案，不得修改代码。
5. 修改前说明准备改哪些文件、每个文件为什么属于该职责层。

### 修改中

- 只修改解决当前问题所需的最小范围。
- 遵守现有命名、格式、类边界和错误处理方式。
- 优先修正已有逻辑，不创建平行状态机、旁路节点或重复配置。
- 新增任何相似功能前，必须先用全仓搜索确认是否已有实现、标志位、计时器、配置项、日志或测试，并判断其唯一职责所有者。
- 如果现有逻辑本身错误，应在确认调用方和行为影响后直接修正或替换；完成替换后必须清理失效的旧实现，不能把新逻辑叠加在旧逻辑旁边继续运行。
- 清理范围包括无效函数、重复分支、旧状态/标志位、旧计时器、废弃参数、重复日志、失效注释和只服务于旧逻辑的测试。删除前必须搜索调用关系，不能误删仍在使用的公共接口。
- 除非存在明确且仍被使用的兼容需求，不保留“旧实现 + 新实现 + 转发包装”结构。确需兼容时必须说明保留期限、调用方和退出条件。
- 每项业务规则只能有一个权威实现。不同层只传递数据或调用结果，不得各自实现一套近似判定。
- 新代码必须简洁、高效且易读：减少不必要的类、函数、状态、数据复制、动态分配和重复计算，不用复杂抽象包装简单逻辑。
- 对 50 Hz 控制循环、点云、地图和轨迹等高频/大数据路径，修改时必须考虑时间复杂度、内存分配、锁范围和日志频率，不能用高开销实现换取表面上的代码方便。
- 新增抽象必须确实减少复杂度或符合已有模式。
- 不做与需求无关的重构、重命名、格式化或依赖升级。
- 发现之前添加的错误、冗余或失效代码时应及时删除，并确认没有有效调用方；不能因为旧代码已经存在就继续堆叠补丁。
- 不编辑 `build_runtime/`、`devel_runtime/`、`install_runtime/`、`dist/` 等生成目录中的文件。
- 不改变 MQTT JSON 字段、话题或状态语义，除非需求明确要求并同步更新协议层、配置、日志和测试。

### 修改后

1. 检查分层依赖，可运行 `tools/check_navdog_layer_boundaries.sh`。
2. 再次全仓搜索被替换的符号、参数、日志关键字和状态名，确认旧逻辑已清理且没有残留调用。
3. 检查 diff，确认没有生成物、调试残留、重复实现或无关改动，并判断代码量是否可以进一步缩减而不损失清晰度。
4. 测试放在被修改模块自己的 `test/` 中，覆盖触发条件、恢复条件和边界值；旧行为被删除时同步删除或更新只验证旧错误逻辑的测试。
5. 明确说明新增、修改和删除了什么，尤其说明被替换的旧逻辑及清理结果。
6. 没有执行编译或设备测试时必须明确说明，不能声称已经验证运行效果。

## 9. 编译、交叉编译与部署

- 默认不要自行执行完整编译或 arm64 交叉编译；用户明确要求后再运行。
- 不删除增量构建目录，不使用 clean build 解决普通问题。
- 构建镜像存在时优先增量构建：

```bash
JOBS=8 SKIP_IMAGE_BUILD=1 ./scripts/build_arm64_runtime_docker.sh
```

- 只有 Dockerfile、基础镜像或依赖环境变化时才重建 builder image。
- `exec format error` 属于宿主机 arm64/QEMU/binfmt 执行环境问题，应检查并修复 binfmt 注册，不应通过反复清理源码构建目录处理。
- 运行包部署到 `~/Hc_work/own/dmq_dog_runtime`；压缩包存放在 `~/Hc_work/dist`。
- 部署脚本应自动创建目录、选择明确的运行包、解压、source ROS 和运行包环境、检查 `dmq_dog`，最后启动 `bringup_real.launch`。
- 通配符匹配多个压缩包时不得无提示地混合解压，应选择最新或明确指定的单个包。

## 10. 验收重点

涉及真实导航的修改至少检查以下行为：

- MQTT 断开、错误 JSON、重复路线、`ctrl=0` 和新任务重启。
- Route/SCAN 模式切换期间只有一个 `/cmd_vel` 发布者和一个有效命令所有者。
- 普通路线跟随流畅，不把每个路点当成独立终点反复对齐。
- 遇障能提前减速、绕障或安全停车，不允许先碰撞再响应。
- 障碍解除后恢复条件明确；不能因旧轨迹、旧 sequence 或未清理标志永久停车。
- 最终到达同时满足距离和方向要求，受阻不误报成功。
- 里程计跳变、雷达超时、规划超时和恢复耗尽时保持安全停车并输出可定位日志。
- 实际 MQTT 速度与规划器验证过的运动模型一致。

## 11. 明确禁止事项

- 禁止把真实狗代码放入 `simulator`。
- 禁止在 Core/Task/Protocol 中引入 ROS。
- 禁止让 Planner 解析 MQTT 或决定任务状态。
- 禁止增加第二个 `/cmd_vel` 发布者。
- 禁止复制模式切换、终点判断或任务状态机到 Runtime/Bridge/Mux。
- 禁止用硬编码魔法数代替 YAML 配置。
- 禁止通过放宽安全阈值掩盖里程计、运动模型或规划问题。
- 禁止在已有同职责实现旁边再增加一套相似函数、状态机、标志位、计时器或配置。
- 禁止只增加新逻辑而保留已确认错误且无调用价值的旧逻辑。
- 禁止以“以后可能用到”为由保留无明确调用方的死代码、兼容包装或废弃参数。
- 禁止在控制循环中引入无必要的数据复制、动态分配、阻塞操作或高频无节流日志。
- 禁止未经分析就同时修改多层代码。
- 禁止自动覆盖用户改动、清理工作区或提交代码。
- 禁止在未编译、未测试时宣称问题已经完全解决。
