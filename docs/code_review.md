# 救援集群系统架构重构与优化建议

针对前期 Code Review 中确认但未能通过简单代码修改解决的三个深层架构问题，在此提供完整的重构与工程落地建议方案。

---

## 1. 跟随逻辑重构 (UWB + LiDAR 多传感器融合)
**问题背景**：目前系统在仿真中依赖理想坐标（或容易产生累积误差的纯里程计）进行跟随。在真实的灾害废墟地形中，轮式/履带里程计滑转率极高，单纯依靠航位推算（Dead Reckoning）会导致跟随机器人迅速丢失领航者（Scout）。

**解决方案设计**：
从“绝对坐标跟随”转变为**“高鲁棒性的相对位姿跟随”**，引入 UWB（超宽带）与 LiDAR（激光雷达）融合。

*   **硬件配置**：
    *   在 Scout（领航）尾部安装高反光率圆柱体（或特定特征板），并在其顶部安装 UWB 锚点（Tag）。
    *   在 Carrier 与 Specialist（跟随）前部安装 2D LiDAR，并配备 UWB 基站（Anchor）。
*   **软件管道 (ROS2)**：
    1.  `uwb_node`: 实时输出与 Scout 的绝对距离 $d$（无漂移，但在有遮挡时存在多径干扰）。
    2.  `lidar_tracker_node`: 对 2D LiDAR 点云进行聚类（Clustering），提取高反特征，输出 Scout 相对于自身的 $(x, y, \theta)$。
    3.  `ekf_relative_fusion`: 使用扩展卡尔曼滤波（EKF），以 UWB 距离作为观测值约束 LiDAR 聚类目标的漂移，在 LiDAR 短暂失效（如烟雾遮挡）时，依赖 UWB 和从机里程计进行航迹推算。
    4.  `pure_pursuit_controller`: 直接订阅融合后的相对位姿，采用纯跟踪算法（Pure Pursuit）或 DWA 生成平滑的 `cmd_vel`。

> [!TIP]
> **落地建议**：优先实现 LiDAR 聚类追踪。可以使用开源库 `leg_tracker` 或 `laser_filters` 进行修改，UWB 仅作为丢失找回的辅助手段。

---

## 2. DDS 风暴解决方案 (QoS 调优与单播发现)
**问题背景**：ROS2 默认的 DDS 配置依赖组播（Multicast）进行节点发现。在 3 台机器人的复杂网络环境（尤其是带有 4G/5G 中继路由器的局域网）下，组播包极易引发网络拥塞（DDS Storm），导致 CPU 满载、丢包率飙升和节点假死。

**解决方案设计**：
摒弃默认配置，**强制采用单播发现（Unicast Discovery）**并对 Topic 实施严格的 QoS 分级。

*   **配置 XML (以 CycloneDDS 为例)**：
    创建一个 `cyclonedds.xml`，将其分发到所有机器人，并设置环境变量 `export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml`。
    ```xml
    <?xml version="1.0" encoding="UTF-8" ?>
    <CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
        <Domain id="any">
            <General>
                <NetworkInterfaceAddress>wlan0</NetworkInterfaceAddress>
                <!-- 禁用组播，防止风暴 -->
                <AllowMulticast>false</AllowMulticast> 
            </General>
            <Discovery>
                <!-- 明确指定集群内机器人的 IP 地址 -->
                <Peers>
                    <Peer address="192.168.8.101"/> <!-- Scout -->
                    <Peer address="192.168.8.102"/> <!-- Carrier -->
                    <Peer address="192.168.8.103"/> <!-- Specialist -->
                </Peers>
                <ParticipantIndex>auto</ParticipantIndex>
            </Discovery>
        </Domain>
    </CycloneDDS>
    ```

*   **代码层面的 QoS 分级调优**：
    *   **控制指令 (`/cmd_vel`, `set_mode`)**: `RELIABILITY=Reliable`, `DURABILITY=Volatile`, `HISTORY=Keep_last(1)`。
    *   **高频传感器 (`/odom`, 热成像图像)**: `RELIABILITY=Best_effort`, `DURABILITY=Volatile`, `HISTORY=Keep_last(1)`。**绝不允许使用 Reliable 传输图像。**

---

## 3. LoRa 缺失的补全设计 (极低带宽灾备通信)
**问题背景**：深入地下废墟时，4G 信号和 Wi-Fi 极易彻底断连。此时系统必须具备灾备通信能力，而 LoRa 虽然带宽极小（0.3kbps - 5kbps），但穿透性极强。

**解决方案设计**：
LoRa 无法承载 ROS2 的 DDS 协议。必须在 ROS2 边缘编写一个纯粹的、高度压缩的 `lora_bridge_node`。

*   **双链路冗余状态机**：
    结合已有的 `relay_manager.py`：当 `/carrier/network_ok` 变为 `False` 时，系统自动挂起当前高带宽任务，进入**“LoRa 灾备模式”**。
*   **通信协议设计 (二进制序列化)**：
    复用项目中现有的 `swarm_protocol.py` (CRC8 校验)，但针对 LoRa 做极致压缩。每一帧不能超过 10 - 20 字节。
    *   **下行 (地面站 → 集群)**：仅发送**心跳**、**全局紧急停止 (E-Stop)**、**强制返航指令**。
    *   **上行 (集群 → 地面站)**：频率降至 `0.5 Hz`。采用分时发送防碰撞（Scout 在 0.0s 发，Carrier 在 0.6s 发）。
    *   上行数据结构示例（总计 12 字节）：
        `[Header(1)][RobotID(1)][Status_Bitmask(1)][Battery_pct(1)][PosX_float(4)][PosY_float(4)][CRC8(1)][Tail(1)]`

> [!IMPORTANT]
> **避免逻辑冲突**：LoRa 节点接收到的控制指令，应通过特定的 ROS topic（如 `/lora/cmd_vel`）接入 `twist_mux`，并赋予**最高优先级**，确保在断网时地面站仍能强切控制权。
