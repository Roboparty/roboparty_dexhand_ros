# roboparty_dexhand_ros

运行在机器人板端的 ROS 2 灵巧手适配包，通过 `roboparty_dexhand` 的
`dexhand_py` 模块控制 RP_Hand 6DOF。提供双手独立节点、初始化/回零服务和
六关节位置命令订阅；不连接 PICO，也不进行手柄映射或手骨骼 retarget。

```text
操作端 roboparty_dexhand_teleop
  -> /dexhand_left/cmd + /dexhand_right/cmd
机器人板端 roboparty_dexhand_ros
  -> dexhand_py -> roboparty_dexhand -> CAN-FD -> RP_Hand
```

本文按“准备依赖 → 构建 → 实机准备 → 实机测试”排列。命令在目标板的
Bash 终端执行。构建不需要连接灵巧手；实机回零和位置命令会产生运动。

## 1. 准备环境

已验证环境：Orange Pi / AArch64、Ubuntu 22.04、系统 Python 3.10、
ROS 2 Humble。其他系统或 ROS 版本尚未在本项目中验证。

如果尚未安装 ROS，请先按
[ROS 2 Humble 官方安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html)
配置 ROS 软件源并安装 `ros-humble-ros-base`。
确保 `/opt/ros/humble/setup.bash` 存在，再安装构建和运行依赖：

```bash
sudo apt update
sudo apt install git python3-colcon-common-extensions python3-setuptools \
  ros-humble-rclpy \
  ros-humble-std-msgs ros-humble-std-srvs \
  ros-humble-launch ros-humble-launch-ros can-utils
```

### 底层驱动（必需）

本包不会自动下载或编译底层驱动。先按
[roboparty_dexhand 的安装说明](https://github.com/Roboparty/roboparty_dexhand#readme)
完成驱动安装，要求版本不低于 **0.4.2**。首次从源码部署，使用该文档的
“从源码构建”流程即可，不需要额外安装 RoboParty 基础包。

这些名称对应不同层次：

| 名称 | 含义 |
| --- | --- |
| `roboparty_dexhand` / `dexhand_py` | 底层驱动项目 / 它提供的 Python 模块 |
| `roboparty-dexhand` | 底层驱动的 Debian 软件包名 |
| `roboparty-base` | Debian 驱动包依赖的基础环境包，不包含在本 ROS 仓库中 |
| `/opt/roboparty` | 多个 RoboParty 软件包共用的安装目录 |
| `roboparty_dexhand_ros` | 当前 ROS 包，调用驱动，不会代替你安装驱动 |

源码安装与 Debian 软件包安装任选一种：

- **源码安装**：下文假定驱动已在 `~/roboparty_dexhand` 完成构建和安装，
  环境脚本为 `~/roboparty_dexhand/setup.bash`，不是 `install/setup.bash`。
- **Debian 软件包安装**：安装 `roboparty-dexhand` 及其依赖 `roboparty-base` 后，下文每处
  `source ~/roboparty_dexhand/setup.bash` 改为 `source /opt/roboparty/setup.bash`。

使用系统 Python，避免 Conda/venv 的 Python 与 ROS 或驱动扩展版本不匹配。
先确认驱动可以导入：

```bash
source /opt/ros/humble/setup.bash
source ~/roboparty_dexhand/setup.bash
python3 -c 'from dexhand_py import HandDriver, HandModel; import dexhand_py; print(dexhand_py.__file__)'
```

源码安装时应输出 `~/roboparty_dexhand/install/` 下的 `dexhand_py*.so` 路径。
如果失败，先解决驱动安装或环境加载问题，再继续。

## 2. 获取源码并构建

首次下载：

```bash
cd ~
git clone https://github.com/Roboparty/roboparty_dexhand_ros.git
```

已有源码可跳过下载。以下直接将仓库目录作为构建工作空间，无需额外建立 `src/`：

```bash
source /opt/ros/humble/setup.bash
source ~/roboparty_dexhand/setup.bash
cd ~/roboparty_dexhand_ros

colcon build --symlink-install --packages-select roboparty_dexhand_ros
source install/setup.bash
```

成功时应看到 `Summary: 1 package finished`。构建成功不代表运行时驱动和
硬件可用，请继续测试。修改源码后可以重复构建命令。

### 每次运行前加载环境

每次新开终端，包括第二个 SSH 会话，都需要重新加载三层环境：ROS 2、
底层驱动、当前 ROS 包。以下两组**按驱动安装方式二选一**。第三条命令仅在
本 ROS 包构建成功后可用。这些命令不会启动节点或让手运动。

**方式 A：底层驱动从源码安装（本文默认）**

```bash
source /opt/ros/humble/setup.bash
source ~/roboparty_dexhand/setup.bash
source ~/roboparty_dexhand_ros/install/setup.bash
```

**方式 B：底层驱动通过 Debian 软件包安装**

```bash
source /opt/ros/humble/setup.bash
source /opt/roboparty/setup.bash
source ~/roboparty_dexhand_ros/install/setup.bash
```

两种方式后续的 launch 命令完全相同。不要在同一终端叠加两套驱动环境；
切换安装方式时新开终端。`~/roboparty_dexhand_ros` 是本文选定的源码和构建目录，
并非固定的系统路径；使用其他工作空间时，将第三条改成该工作空间的
`install/setup.bash`，不要同时加载同名 ROS 包的另一份安装。

不确定终端实际加载了哪份安装时，可以检查：

```bash
python3 -c 'import dexhand_py; print(dexhand_py.__file__)'
ros2 pkg prefix roboparty_dexhand_ros
```

源码驱动应来自 `~/roboparty_dexhand/install/`，Debian 驱动应来自
`/opt/roboparty/`；ROS 包应来自 `~/roboparty_dexhand_ros/install/`。

### 清理后重新构建

需要完全重新构建时，在新开的终端执行以下命令，再重复上面的构建步骤。
此操作只删除当前仓库的构建、安装产物和 colcon 日志：

```bash
cd ~/roboparty_dexhand_ros
rm -rf -- build install log
```

## 3. 实机准备

厂商 SDK 的接收 callback 是进程级资源，因此每个进程只管理一只手。
双手 launch 默认创建两个进程：

| 节点 | 默认 CAN 接口 | 默认 node ID |
| --- | --- | --- |
| `dexhand_left` | `can0` | `1` |
| `dexhand_right` | `can3` | `1` |

CAN 接口名属于部署配置，请按实际接线修改。两个 node ID 相同的设备应位于
不同 CAN 总线。当前 launch 的 `node_id` 参数同时用于左右手。

在启动节点前，完成底层驱动文档中的
[CAN-FD 设置](https://github.com/Roboparty/roboparty_dexhand#linux-can-fd-设置)和
[首次反馈周期配置](https://github.com/Roboparty/roboparty_dexhand#首次配置反馈频率)。
本包不会设置 CAN 速率或持久化反馈周期。已经配置好的设备无需每次重做。
参考部署采用名义速率 1 Mbps、数据速率 5 Mbps、20 ms 反馈周期。

```bash
ip -details link show can0
ip -details link show can3
```

两个接口应处于 `UP`，显示 CAN-FD（`<FD>`），且速率与实际设备匹配。
停止占用同一只手的其他控制程序，再继续测试。

## 4. 实机测试

### 只初始化通信，不使能、不回零

先在当前终端加载第 2 节选择的三层环境，再执行：

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py \
  left_interface:=can0 right_interface:=can3 node_id:=1 \
  home_on_start:=false
```

预期左右手各出现：

```text
hand detected and initialized: initialized (enable=False, home=False)
```

这表示驱动初始化返回成功，不代表完成了真实运动或反馈验收。
此模式会向硬件发送初始化配置，但不会使能或回零。
按 `Ctrl+C` 结束，正常退出显示 `process has finished cleanly`。

只有一只手时，停止双手 launch，改用以下命令（左手接 `can0`，不使能、不回零）：

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py \
  left:=true right:=false left_interface:=can0 node_id:=1 \
  home_on_start:=false
```

只启动右手时使用 `left:=false right:=true right_interface:=can3`。
每次只运行一个对应的 launch，避免多个进程同时控制同一只手。

### 使能并回零

停止上一条 launch 后执行。**该命令会使双手运动**，先确保运动范围内没有
障碍物，并准备好硬件停止手段：

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py \
  left_interface:=can0 right_interface:=can3 node_id:=1 \
  velocity:=8000 home_on_start:=true
```

预期左右手各出现 `initialized (enable=True, home=True)`，并观察实际回零结果。
`home_on_start` 默认是 `true`，省略该参数也会使能和回零。

保持 launch 运行，在另一个已加载三层环境的终端查看接口：

```bash
ros2 node list
ros2 topic info /dexhand_left/cmd
ros2 topic info /dexhand_right/cmd
ros2 service list
```

应存在两个节点、两个 `std_msgs/msg/Int32MultiArray` 命令话题，各有一个
订阅者，以及左右手的 `init/reset` 服务。

## ROS 接口与参数

| 接口（右手将 `left` 替换为 `right`） | 类型 | 行为 |
| --- | --- | --- |
| `/dexhand_left/cmd` | `std_msgs/msg/Int32MultiArray` | 六个整数依次对应关节 1–6，范围 `0..10000`，单位为编码器计数；收到后执行运动 |
| `/dexhand_left/init` | `std_srvs/srv/Trigger` | 重新创建驱动并使能，不回零；适用于零位已知的恢复场景 |
| `/dexhand_left/reset` | `std_srvs/srv/Trigger` | 重新创建驱动、使能并回零，会运动 |

例如，在节点运行期间使左手重新回零：

```bash
ros2 service call /dexhand_left/reset std_srvs/srv/Trigger '{}'
```

检查返回的 `success: true`；调用成功与话题存在是不同的验证步骤。
未使能时仅发布 `cmd` 不会自动使能。位置命令必须包含恰好六个合法整数。
关节与手指的对应关系、目标值和速度应按实际机构确认。

| launch 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `left` / `right` | `true` | 是否启动对应的节点 |
| `left_interface` / `right_interface` | `can0` / `can3` | 实际连接灵巧手的 CAN 接口 |
| `node_id` | `1` | 两只手共用的配置值，合法范围为 `1..127` |
| `home_on_start` | `true` | 启动时使能并回零；`false` 仅初始化通信 |
| `velocity` | `8000` | 位置运动速度，单位为计数/秒 |

可用以下命令查看 launch 参数，不会启动节点：

```bash
ros2 launch roboparty_dexhand_ros dual_dexhand.launch.py --show-args
```

## 常见问题

| 现象 | 检查方法 |
| --- | --- |
| `colcon: command not found` | 安装 `python3-colcon-common-extensions` |
| 找不到 `rclpy` | 加载 `/opt/ros/humble/setup.bash`，检查是否误用了虚拟环境中的 Python |
| 找不到 `dexhand_py` 或加载 `.so` 失败 | 加载正确的驱动环境脚本；检查 Python 版本、CPU 架构及完整驱动安装目录 |
| `Package 'roboparty_dexhand_ros' not found` | 确认构建成功，并在当前终端加载该仓库的 `install/setup.bash` |
| 初始化失败并反复重试 | 检查手的供电、接线、CAN 接口、速率、node ID、反馈周期和是否有其他控制进程 |
| `home_on_start:=false` 后手不动 | 这是未使能状态；需要正常使能/回零后才能测试运动 |
| 回零失败 | 检查实际位置、报警与反馈配置，按底层驱动的硬件验收说明排查 |

## 当前限制

本包目前只提供命令订阅和生命周期服务，尚未发布带时间戳的位置、状态、
电流、报警或 `/joint_states` 反馈。底层驱动支持读取这些数据，但本 ROS
节点尚未将其发布，因此不能仅凭 ROS 话题完成运动反馈验收或 rosbag 采集。
底层硬件验收见
[roboparty_dexhand/VALIDATION.md](https://github.com/Roboparty/roboparty_dexhand/blob/HEAD/VALIDATION.md)。

## License

本包采用 [GPL-3.0](LICENSE)。底层驱动及厂商 SDK 的许可说明见对应项目。
