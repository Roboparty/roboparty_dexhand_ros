# roboparty_dexhand_ros

`roboparty_dexhand_ros` 是运行在机器人板子上的 ROS 2 设备适配包。它调用
`roboparty_dexhand` 提供的 `dexhand_py`，负责灵巧手生命周期与 CAN-FD
执行；它不连接 PICO，也不进行手柄映射或手骨骼 retarget。

## 系统边界

```text
操作端 roboparty_dexhand_teleop
  -> /dexhand_left/cmd + /dexhand_right/cmd

机器人板子 roboparty_dexhand_ros
  -> dexhand_py
  -> roboparty_dexhand
  -> CAN-FD
```

厂商 SDK 的接收 callback 是进程级资源，因此每个节点进程只管理一只手。
双手 launch 默认启动：

- `dexhand_left`：`can0`、node ID 1；
- `dexhand_right`：`can3`、node ID 1。

CAN 分配属于机器人部署配置，不会暴露到操作端话题名称中。

## 构建

板端需要先安装 `roboparty-dexhand` Debian 包，然后在 ROS 2 工作空间中构建：
要求驱动版本不低于 `0.4.2`，该版本会用新鲜位置和状态反馈确认真实回零。

```bash
source /opt/ros/humble/setup.bash
source /opt/roboparty/setup.bash
colcon build --symlink-install --packages-select roboparty_dexhand_ros
source install/setup.bash
```

## 启动

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py \
  left_interface:=can0 right_interface:=can3 node_id:=1
```

默认启动时使能并回零双手。只连接设备、不立即驱动时使用：

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py \
  home_on_start:=false
```

节点接口：

```text
/dexhand_left/cmd    /dexhand_right/cmd
/dexhand_left/init   /dexhand_right/init
/dexhand_left/reset  /dexhand_right/reset
```

## 数据采集边界

本包是手部真实反馈的唯一权威来源。当前节点已能通过 `dexhand_py` 读取位置、
状态、电流和报警，但尚未将这些缓存发布为 ROS 状态话题。接口阶段需要在本包
补充带时间戳的左右手状态输出，供 rosbag 与全身的 `/joint_states`、`/action`
和 `/imu` 同步记录。
