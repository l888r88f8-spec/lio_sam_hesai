# 室外定位测试（简版）

在**之前漂移的室外位置**测试，使用 `hesai_outdoor_yifu`。保持同一位置和朝向，先只启动传感器与定位。

## 1. 准备保存目录

所有测试终端执行，ROS 域沿用实机驱动的设置：

```bash
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
mkdir -p /home/robot/localization_tests
cd /home/robot/localization_tests
```

## 2. 检查雷达频率

定位启动前测一次，启动后再测一次；每次观察约 30 秒后按 Ctrl+C：

```bash
ros2 topic hz /lidar_points --window 100 \
  2>&1 | tee "lidar-hz-$(date +%Y%m%d-%H%M%S).log"
```

目标接近驱动设定的 **10 Hz**。启动后同时看 `LOC_HEALTH.lidar_rate_hz`：

- 命令接近 10 Hz、健康统计明显更低：重点查定位节点接收。
- 两者都明显不足：继续查驱动、传输和整机负载；不能直接判定硬件低频。

## 3. 通过 web_control 启动定位

通过网页选择 `hesai_outdoor_yifu` 并启动定位，第一轮使用分块地图配置。

**定位终端输出由 web_control 自动保存**，记录本轮日志文件名／时间段即可：

```text
/home/robot/.local/state/web_control/logs/localization/
```

原始点云和 IMU 的 bag 仍需单独录制。节点启动后，在准备好的终端保存实际参数并录包：

```bash
ros2 param dump /lio_sam_hesai_relocalization > "params-$(date +%Y%m%d-%H%M%S).yaml"
ros2 bag record -o "raw-$(date +%Y%m%d-%H%M%S)" /lidar_points /imu
```

录包覆盖初始化和异常前后，建议约 2 分钟，结束按 Ctrl+C。记录是否在录包后出现降频。

## 4. 静止，再行驶

1. 车保持静止，手动给**一次**位置，确认 `LOC_INIT commit accepted`。
2. 静止观察 **60～90 秒**：位置是否漂移、估计速度是否接近零。
3. 静止正常后，缓慢前进、停车、转向：位置是否跟随，停车后是否继续漂移。
4. 异常时记录**时间、车实际有没有动、完整报错**，先保留首次异常，不连续重新给位置。

## 5. 换全局地图对照

通过网页停止本轮定位，沿用之前切换地图的方式，将定位地图路径切换为以下文件后再启动：

```text
/home/robot/map/hesai_outdoor_yifu/GlobalMap.pcd
```

从保存的实际参数确认地图路径确已切换。保持两轮初始化方式、自动重定位开关一致，重复第 2～4 步；开关以实际参数为准。每轮注明**分块／全局、静止／行驶、首次异常时间**。

**测试结束保留本轮 web_control 日志，以及 `/home/robot/localization_tests` 中的测量、参数和 bag。** 重点看 `LOC_TIMEOUT`、`LOC_TRACK match_failed`、`LOC_HEALTH` 状态转换和完整 TF 报错。

注意：当前健康标称频率为 10 Hz，与驱动设置一致；日志摘要约每秒打印一次，不代表定位只有 1 Hz。
