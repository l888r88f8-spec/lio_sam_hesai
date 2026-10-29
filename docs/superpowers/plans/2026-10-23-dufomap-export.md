# DUFOMap 导出时动态清理计划

**目标**：只在导出 `/lio_sam/save_map(_options)` 时，用官方 DUFOMap 对完整关键帧扫描做射线证据分类，过滤 Corner/Surf（Global 由其组成）；地面是独立分割结果，完全不经过 DUFOMap 分类。保留原始点坐标和 intensity，`map_save.resolution` 仍只控制最终密度。默认关闭，不改变实时 SLAM 和既有导出。

**架构**
- 会话期缓存：启用 `dufomap_export.enable` 后，在独立临时目录逐关键帧写 `cloud_deskewed`（雷达局部坐标、完整 XYZI）。不进入导出目录、不长期常驻内存。
- 导出快照：同锁复制最终关键帧位姿、corner/surface 特征和现有 `groundCloudKeyFrames`；用最终位姿生成 4x4 矩阵。回环后的位姿自动生效，ground 与 global 共用该快照。
- 数据集：导出到 `<destination>/dufomap_export/`（manifest、`scans/`、`targets/`、`filtered/`、日志、summary），可独立复跑。
- 官方后端：独立 Python 工具逐帧 `dufomap.run(..., cloud_transform=True)`，`oncePropagateCluster(if_propagate=True, if_cluster=False)`，再对目标点调用官方 `segment(points, pose, cloud_transform=False)` 取 0/1 标签；只保留 0。目标 PCD 按原始结构化字段过滤，不输出体素质心。
- 服务集成：C++ 用 `posix_spawn` 传 argv（不经 shell），失败/缺依赖/缺帧/非零退出均返回保存失败；成功后才写最终地图并生成瓦片。
- 地面顺序：DUFOMap 不接收 ground target。导出时从最终位姿快照重建未过滤地面，再沿用原补地面栅格 → 下采样 → flatten/shift 流程；`save_ground_map` 开关保持请求语义。DUFOMap 与旧 `static_map_filter` 互斥，导出时明确报错，避免旧滤波提前合并点。
- 密度：DUFOMap 的 `resolution/d_s/d_p` 只服务证据分类；最终密度仍由请求的 `resolution` / `ground_resolution` 控制。

**文件**
- 新增 `include/common/dufomap_export.h`、`src/common/dufomap_export.cpp`：配置校验、manifest 序列化、工具路径解析、无 shell 子进程与超时。
- 修改 `include/common/slam_config.h`、`src/common/slam_config.cpp`、`config/mapping.yaml`（只追加默认关闭的 `dufomap_export` 块，不动用户已有值）。
- 修改 `src/mapOptimization.cpp`：扫描缓存、导出快照、数据集、corner/surface 过滤、地面原流程、瓦片顺序。已删除仅为 ground 分类新增的 raw ground 通道和 `CloudInfo` 字段。
- 新增 `scripts/dufomap_export_filter.py`：轻量 PCD I/O（保留所有字段）+ 官方 DUFOMap 后端。
- 新增测试：`test/test_dufomap_export_filter.py`（假后端单测 + 真实后端可选 smoke）、C++ 契约/集成测试；扩展 `test/test_map_optimization_save_export.cpp`。
- 修改 `CMakeLists.txt`、`package.xml`、新增使用文档。

**TDD 顺序**
1. 先写 Python 工具测试：原始点/intensity/额外字段保留、缺帧、缺依赖、后端异常、姿态与原点传入、目标点标签过滤。
2. 写 C++ 契约测试：manifest/参数、argv、shell 元字符安全、超时/非零退出。
3. 写导出集成测试：禁用路径不变、启用但缺缓存失败且不落未清理地图、假工具成功、旧 static filter 冲突报错、回环后位姿/原点快照。
4. 实现最小代码使测试转绿。
5. 真实 DUFOMap smoke：静态墙/遮挡/移动物体。当前机器为 aarch64，PyPI 官方 wheel 只提供 x86_64；先尝试安装，若不可用则用官方 C++ 构建做算法 smoke，并如实记录限制。

**验证命令**
- `python3 -m pytest -q test/test_dufomap_export_filter.py`
- `cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target test_dufomap_export test_map_optimization_save_export -j2`
- 对应 `ctest` / 直接运行测试二进制；只编译受影响 target。
- 记录每条命令退出码、日志路径；不伪称未执行的验证。

**约束**
- 不提交 git，不改 `launch/path.py`，不恢复被删文件，不写 `/map`，测试只用隔离临时目录。
- 保留 `config/mapping.yaml` 用户 diff 和原值。

## 2026-10-23 实施与验证记录

- ARM64 后端采用固定官方 DUFOMap/UFOMap C++ 核心的薄 pybind 绑定；不是 PyPI 的预编译 wheel，也不是算法替身。安装器与原生源码可部署，下载/构建/venv 位于仓库之外。summary 记录 backend kind、核心来源和 revision。
- 分类网格不合并输出点，仍影响删点范围；最终降采样由 map_save/服务请求控制。真实地面通道直接保留原始测量 XYZ/intensity，不提前 VoxelGrid 或压平。官方串行 miss 分支未实现，因此拒绝 num_threads=1；d_p 限制 0–4。
- 修复了骨架编译错误、重复 filtered/ 路径、缺地面截断特征、合法空 PCD、缓存失败空错误指针和 PCL 写入异常。成功验证后才替换旧最终地图；失败重试不会读旧过滤结果。缓存只在最后保存完整成功且没有新增关键帧时于退出清理。
- 默认 worker 同时复制到 build 和 install 可执行目录，覆盖 AMENT_CMAKE_SYMLINK_INSTALL 下 `/proc/self/exe` 解析到 build 的布局。

红测证据（日志根目录 `/tmp/lio_sam_dufomap_validation`）：

- `python3 test/test_dufomap_export_filter.py`：新增 d_p=5、num_threads=1 拒绝测试先失败 2 次（`red_python.log`），修复后通过。
- `test_dufomap_export`：参数拒绝测试先失败（`red_contract2.log`），修复后 10/10 通过（`green_contract.log`）。
- `test_map_optimization_save_export`：基线 10 项中 8 项通过，失败退出缓存被删、PCL 拒绝合法空点云两项失败（`red_export.log`）；修复及扩展后 15/15 通过，包含 PCL 写盘异常、同锁最终位姿与局部射线扫描、特殊字符路径、二次失败保留旧图、过滤后地面补点、缓存生命周期及真实 native 后端调用链（`final_export.log`）。

已执行绿测：

```bash
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/lio_sam_dufomap_validation/ros_logs
export LIO_SAM_DUFOMAP_TEST_PYTHON=/home/robot/.local/share/lio_sam_hesai/dufomap-venv/bin/python
/home/robot/slam_ws/build/lio_sam_hesai/test_map_optimization_save_export
/home/robot/slam_ws/build/lio_sam_hesai/test_dufomap_export
/home/robot/slam_ws/build/lio_sam_hesai/test_slam_config
/home/robot/.local/share/lio_sam_hesai/dufomap-venv/bin/python test/test_dufomap_export_filter.py
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_dufomap_export|test_dufomap_export_filter|test_map_optimization_save_export|test_save_map_request|test_map_artifact_layout)$' --output-on-failure
```

结果：真实 ExportMap 集成 15/15；进程/manifest 合同 10/10；配置 33/33；真实 Python 后端全套 10/10、无 skip；相关 CTest 5/5。`green_python_real2.log` 记录真实 Python 场景，静态墙保留、历史动态位置删除、遮挡未知点保留，所有额外 PCD 字段保留。工具替身测试仅证明接口/失败处理，不作为算法效果证据。

节点测试在受限沙箱下有 FastDDS 网络接口/UDP 权限日志，测试本身通过且不依赖 ROS 网络通信。ARM 编译还出现既有 GCC ABI 注记及 OpenCV/TBB 链接警告。未调用真实 `/map` 导出；所有集成输出在测试临时目录。保留原用户 mapping.yaml 注释/空白行、launch 文档删除与 path.py；没有提交或创建 worktree。

最终构建与安装验收（所有命令退出码 0）：

```bash
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_dufomap_export test_map_optimization_save_export test_slam_config \
  lio_sam_hesai_imageProjection lio_sam_hesai_featureExtraction \
  lio_sam_hesai_mapOptimization -j2
cmake --install /home/robot/slam_ws/build/lio_sam_hesai
source /home/robot/slam_ws/install/setup.bash
ros2 run lio_sam_hesai install_dufomap_backend.sh --smoke
ros2 run lio_sam_hesai dufomap_export_filter.py --help
```

- `final_build.log`：三个相关节点 Release 构建及测试 fresh build 成功。另按目标重建全部 CloudInfo C/C++/Python 类型支持（`message_build.log`）。
- 后续调整（ground 不参与 DUFOMap）：删除 `cloud_ground_raw` 及 raw ground 关键帧通道，`CloudInfo` 恢复原 schema。本项改动后须重新编译消息包和地图节点，再重跑上述构建/测试记录；旧 record 中的 raw ground 字段验证已不再适用。
- `install.log`：安装成功。`installed_validation.log`：worker、installer、native CMake/C++/ARM64 shim/smoke 文件均存在；安装可执行 symlink 解析到 build 时默认 worker 可达。
- `installed_backend.log`：从实际 ros2 run 安装入口运行真实 backend smoke，`passed: true`；线程 2/auto 0 通过，单线程显式拒绝。安装器按源码路径使用独立构建缓存，支持仓库和安装目录间重定位。
- `final_ctest.log`：增加配置用例后最终 CTest **6/6**，0 失败。`final_python_real.log`：最终真实 Python 全套 **10/10**，无 skip；`installed_real_export.log`：安装器重新构建后真实 ExportMap→Python worker→native backend→PCD 回读用例 **1/1** 再次通过。
- `git diff --check` 唯一报告为 mapping.yaml 原用户修改中的空白行 trailing whitespace，依约保留；未修改用户 path.py 或恢复用户删除的文档。

## 2026-10-24 取消独立 ground 分类：验证记录

本轮实现取代前述独立 ground target / raw ground 通道方案：DUFOMap 导出只分类 corner 和 surface；GroundMap 使用原有地面保存链路，保留补点、压平、分辨率和保存开关行为。完整关键帧扫描自然包含的地面回波继续参与射线证据，不把分割地面额外拼入扫描。CloudInfo 恢复原 schema，不包含 cloud_ground_raw。

验证日志根目录：`/tmp/lio_sam_dufomap_ground_passthrough_validation`。

执行命令（ROS Humble，标准工作区增量构建，未使用镜像产物代替标准构建）：

```bash
source /opt/ros/humble/setup.bash
cmake -S /home/robot/slam_ws/src/lio_sam_hesai -B /home/robot/slam_ws/build/lio_sam_hesai
cmake --build /home/robot/slam_ws/build/lio_sam_hesai --target \
  test_map_optimization_save_export test_dufomap_export test_slam_config \
  lio_sam_hesai_imageProjection lio_sam_hesai_featureExtraction lio_sam_hesai_mapOptimization \
  lio_sam_hesai__rosidl_typesupport_fastrtps_cpp \
  lio_sam_hesai__rosidl_typesupport_introspection_cpp \
  lio_sam_hesai__rosidl_typesupport_c__pyext \
  lio_sam_hesai__rosidl_typesupport_fastrtps_c__pyext \
  lio_sam_hesai__rosidl_typesupport_introspection_c__pyext -j2
export ROS_LOG_DIR=/tmp/lio_sam_dufomap_ground_passthrough_validation/ros_logs
export LIO_SAM_DUFOMAP_TEST_PYTHON=/home/robot/.local/share/lio_sam_hesai/dufomap-venv/bin/python
/home/robot/.local/share/lio_sam_hesai/dufomap-venv/bin/python test/test_dufomap_export_filter.py
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_dufomap_export|test_dufomap_export_filter|test_map_optimization_save_export|test_save_map_request|test_map_artifact_layout|test_slam_config)$' --output-on-failure
cmake --install /home/robot/slam_ws/build/lio_sam_hesai
source /home/robot/slam_ws/install/setup.bash
python3 /tmp/lio_sam_dufomap_ground_passthrough_validation/verify_installed_cloud_info.py
ros2 run lio_sam_hesai dufomap_export_filter.py --help
```

结果：

- 指定三个节点的 Release 构建、三个测试目标及恢复 schema 后的所有 C/C++/Python 消息类型支持均构建成功，退出码 0（`build.log`）。
- 首轮保存链路 16 项中 15 项通过，唯一失败是新增测试期望 flatten_value=0.1，却未显式设置该值，实际默认值为 0。仅在测试 fixture 添加 `ground_segmentation.flatten_value=0.1`，生产默认值和生产代码未修改；测试目标重建成功（红测 `export.log`、重建 `test_rebuild.log`）。
- 最终 CTest **6/6 通过、0 失败**（`ctest.log`、`ctest_details.log`）。保存链路 **16/16**，包含真实 ExportMap→Python worker→native 后端→PCD 回读；新增仅 corner/surface 分类、保留地面与完整扫描、地面 patch/flatten/save 开关回归均通过。配置 **33/33**，DUFOMap 合同 **10/10**。
- 真实 venv Python 套件 **11/11 通过，无 skip**（`python_real.log`）；没有重装或修改原生后端。
- 安装退出码 0（`install.log`）。安装后的 CloudInfo 字段列表没有 `cloud_ground_raw`，325 字节序列化/反序列化及各字段对比通过（`installed_cloud_info.log`）。复用的 Harness 验证脚本曾把 list 与 array 直接比较导致误判，仅在 /tmp 副本将两边都转为 list，数值验证随后通过，仓库生产代码未因此改变。
- 安装 worker、实际可执行 symlink 解析后的默认 build worker 均与源码逐字节一致；安装后的消息定义同样无 raw 字段（`installed_worker.log`）。实际 `ros2 run` worker `--help` 退出码 0（`installed_help.log`）。
- 检查确认无遗留 mirror 编译进程；本轮 Harness 生成的 `.validation/2026-10-24-dufomap-ground` 完整归档到日志根目录 `harness_previous/`，仓库未残留该验证目录。

没有启动真实 SLAM 节点、调用真实地图保存服务、导出 /map、提交或创建工作树。所有测试输出位于临时目录。用户原有修改保留；`git diff --check` 仍仅报告原 mapping.yaml 空白行 trailing whitespace，依约未改。
