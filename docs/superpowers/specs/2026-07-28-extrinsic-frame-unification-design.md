# 子项目 B：坐标与外参基准统一设计

**日期：** 2026-07-28

**状态：** 已确认

**范围：** 建图与重定位静态 TF 基准统一

## 1. 目标

以已确认的实测安装量为 canonical 基准，统一建图和重定位 launch 发布的
`base_link -> hesai_lidar` 与 `base_link -> imu_link` 静态变换，并通过端到端
断言锁定它们与 `calibration.*` 的方向和数值关系。

canonical 安装量为：

- `T^base_lidar.translation = [0.02, 0.0, 0.32] m`
- `T^base_imu.translation = [0.02, 0.0, 0.22] m`
- 两个静态变换的旋转均为单位阵
- `T^imu_lidar = inverse(T^base_imu) * T^base_lidar`
- 因而 `T^imu_lidar.translation = [0.0, 0.0, 0.10] m`

## 2. 当前状态

- `launch/localization.launch.py` 和
  `launch/localization_with_nonground.launch.py` 已使用 canonical 安装量。
- `config/mapping.yaml` 和 `config/relocalization.yaml` 已共享
  `calibration.translation = [0.0, 0.0, 0.10]`，旋转为单位阵。
- `launch/mapping.launch.py` 仍发布旧值：
  - `base_link -> hesai_lidar = [0.0, 0.0, 0.405]`
  - `base_link -> imu_link = [0.0, 0.0, 0.305]`
- 现有 `test_lidar_imu_calibration` 只对两个重定位 launch 做静态 TF
  端到端断言，未覆盖 mapping launch。

## 3. 设计

### 3.1 Mapping launch 对齐

仅修改 `launch/mapping.launch.py` 中两个静态变换的平移：

- `base_link -> hesai_lidar` 改为 `[0.02, 0.0, 0.32]`
- `base_link -> imu_link` 改为 `[0.02, 0.0, 0.22]`

保持旋转、frame 名称、节点启动方式、`map -> odom` 静态变换和其他 launch
行为不变。mapping launch 继续默认发布静态 TF，不新增
`publish_static_tf` 参数。

### 3.2 标定语义

两份 YAML 的 `calibration.translation`、`calibration.rotation` 和
`calibration.rpy` 保持不变。其 canonical 语义继续为：

```text
T^imu_lidar = inverse(T^base_imu) * T^base_lidar
```

重定位直接消费 `T^imu_lidar`；建图侧使用现有派生方向，不在本子项目调整任何
算法消费者。

### 3.3 端到端测试

扩展 `test/test_lidar_imu_calibration.cpp` 的静态 TF 契约：

1. 对 mapping、localization、localization_with_nonground 三个 launch
   解析 `base_link -> hesai_lidar` 和 `base_link -> imu_link`。
2. 断言三个 launch 的两个静态变换均等于 canonical 安装量和单位旋转。
3. 对每个 launch 计算
   `inverse(T^base_imu) * T^base_lidar`，断言其平移和旋转等于两份 YAML
   共享的 `T^imu_lidar`。
4. `publish_static_tf` 默认值只对两个已有该参数的重定位 launch 继续验证；
   mapping launch 不新增该参数。

## 4. 测试策略

遵循 RED → GREEN：

1. 先把 mapping launch 纳入现有端到端断言。
2. 在旧 mapping 数值下运行 `test_lidar_imu_calibration`，确认因
   `T^base_lidar` 与 canonical 不一致而失败。
3. 仅修改 mapping launch 的两组平移。
4. 重跑定向测试，确认转为 GREEN。
5. 运行完整包构建与全部非历史-lint测试。

## 5. 非目标

- 不在线估计或自动标定外参。
- 不修改 YAML 的 canonical 数值或旋转矩阵。
- 不为 mapping launch 新增静态 TF 开关。
- 不迁移静态 TF 到 URDF，也不删除 launch 内静态 TF。
- 不修改 frame 名称、话题、ROS 参数或算法坐标变换实现。

## 6. 验收标准

- 三个 launch 的 `base_link -> hesai_lidar` 均为
  `[0.02, 0.0, 0.32]` 和单位旋转。
- 三个 launch 的 `base_link -> imu_link` 均为
  `[0.02, 0.0, 0.22]` 和单位旋转。
- 三个 launch 派生的 `T^imu_lidar` 均与两个 YAML 的 calibration 契约一致。
- 定向测试在旧 mapping 值上失败，在新值上通过。
- `lio_sam_hesai` 构建成功，全部非历史-lint测试无回归。

## 7. 风险与回退

本改动只更新 mapping launch 的两个静态平移，并扩展已有契约测试。主要风险是
现有部署仍依赖旧 mapping 基准；该风险已通过用户确认实测 canonical 安装量消除。
若现场安装改变，应同时更新两个静态变换和 calibration，并由端到端测试验证相对
方向，而不是单独修改某一个值。
