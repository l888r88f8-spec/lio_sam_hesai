# DUFOMap 收尾结果

## 修改

- `src/common/slam_config.cpp`
  - 移除启动时按 `dufomap_export.enable && static_map_filter.enable` 的互斥
    `throw`。
  - 保留启动时的 DUFOMap 参数校验；`static_map_filter` 冲突只在
    `ExportMap()` 按本次请求的有效 `dufomap_mode` 检查。
- `include/common/dufomap_export.h`、`src/common/dufomap_export.cpp`
  - 删除仅供启动检查使用、现已无调用的
    `ValidateDufomapFilterCompatibility(const DufomapExportConfig&, ...)`
    重载；保留有效模式布尔版本并补充语义注释。
- `test/test_map_optimization_save_export.cpp`
  - 恢复 10 个真实 C++ 回归测试：
    `DufomapKeepsGroundPatchFlattenAndSaveSwitch`、
    `DufomapSuccessfulDatasetAllowsCacheCleanup`、
    `DufomapNewKeyframesAfterExportRetainCache`、
    `DufomapPcdWriteExceptionsAreReportedAsFailure`、
    `CurrentExportWithoutDufomapUsesOriginalProjectionAndFinalPose`、
    `PatchworkExportWithoutDufomapUsesCompleteScansAndFinalPoses`、
    `CurrentExportPriorityDoesNotRequirePatchworkInput`、
    `MissingOrCorruptGroundCacheFailsBeforeReplacingOldArtifacts`、
    `GroundCacheErrorDoesNotBlockDufomapWhenGroundSaveIsOff`、
    `BothGroundModesOffNeedNoCacheAndWriteNoGround`。
  - 增加共享真实 fixture/helper：4x10 current 投影、Patchwork 完整扫描、
    真实缓存写入、最终位姿、追加关键帧；未使用 fake Python worker。
  - 更新互斥断言：允许启动；`config`/`on` 在启用 legacy filter 时拒绝，
    `off` 正常保存。
- `test/test_dufomap_export.cpp`
  - 互斥单测改为传有效模式；显式 `off` 在两者配置均开启时允许。
- `CMakeLists.txt`
  - `DUFOMap launched Python...` 改为 legacy manual replay 说明。
- `docs/dufomap-export.md`
  - 明确服务接口类型变化后所有调用端必须重新构建；省略
    `dufomap_mode` 继续继承配置默认值。
- `src/common/dufomap_backend.cpp`
  - 调整 include 顺序为 Google C++ 风格。
- `config/mapping.yaml`
  - 仅删除一行行尾空格；未改任何配置值（`enable:true`、`ring_plane
    online`、`patchworkpp off`、`tile_size_m:40` 保持原样）。

## 已验证（轻量）

```text
STATIC CHECK test_map_optimization_save_export.cpp (-fsyntax-only)  exit=0
test_dufomap_native                                                  6/6 PASS
test_dufomap_export                                                 11/11 PASS
test_ground_mode_config                                             12/12 PASS
test_save_map_request                                               20/20 PASS
```

说明：`test_ground_mode_config` 首次退出因沙箱不允许写
`~/.ros/log`，重定向 `ROS_HOME` 到工作区后全部通过；不是代码失败。
`test_map_optimization_save_export` 本次只做语法检查，未编译/运行，以避免
再次触发耗时全构建。

## 待运行完整构建与测试

```bash
source /opt/ros/humble/setup.bash
cd /home/robot/slam_ws
colcon build --packages-select lio_sam_hesai \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
colcon test --packages-select lio_sam_hesai --ctest-args -R \
  'test_(map_optimization_save_export|dufomap_native|dufomap_export|ground_mode_config|save_map_request)'
colcon test-result --verbose
```

若沿用本次轻量目录（`.tmp_colcon`），把构建/安装参数改为
`--build-base .tmp_colcon/build --install-base .tmp_colcon/install`，并在运行
测试前设置可写日志目录：

```bash
export ROS_HOME="$PWD/.dufomap_validation/ros_home"
export ROS_LOG_DIR="$ROS_HOME/log"
```

未提交、未 reset、未写 `/map`、未运行真实节点。
