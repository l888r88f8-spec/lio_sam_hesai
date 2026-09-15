# Shutdown Auto-Save Removal Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 删除 Ctrl+C 退出时的地图自动保存路径、它独占的参数/产物契约，并把“不可回读的退出保存”改为启动期 fail-closed 拒绝加迁移提示，使 `/lio_sam/save_map` 成为唯一地图导出入口。

**Architecture:** Phase 0 先确认代码基线 `c9aa44a` 与工作区状态，然后把本计划文件用 `git add -f` 单独提交为受跟踪工件（`docs: plan shutdown auto-save removal`），再在 `c9aa44a` 上固定构建、门禁、基线扫描事实和旧二进制阳性 smoke。Task A 用 TDD 删除运行期关闭路径（`save_map_enabled_`、`visualizeGlobalMapThread()` 尾部导出代码）与仅供该路径使用的 `MapArtifactLayout::Saved*Cloud()` API，本任务内自洽可编译。Task B 再用 TDD 删除 `MapSaveConfig::enabled`、读取语句、YAML leaf 与 `savePCD` 别名契约，并在任何 `Load*` 之前加原始 override fail-closed guard，同时补 YAML/launch 反向契约。Phase 1 用一次性 `rg`/`grep` allowlist 扫描加注入自检、新旧二进制对照 smoke（固定路径 + marker 文件）、fail-closed 冒烟、`/lio_sam/save_map` 服务接口 smoke 和最终门禁收口，最后更新 `docs/PROJECT_STATUS.md`。本轮不新增任何脚本。

**Tech Stack:** C++17、ROS 2 Humble `rclcpp`、GoogleTest/CTest、PCL、ROS 2 参数 YAML、Bash（`rg`、`ros2` CLI、`ctest`）

---

## 0. 基线与硬约束

**基线：** 代码基线是 `c9aa44a docs: design shutdown auto-save removal`（当前 `main` HEAD，工作区干净）。本轮在它之上的第一个提交是 Phase 0 的计划提交 `docs: plan shutdown auto-save removal`，该提交只新增本计划文件；所有实现与文档改动都发生在它之后。因此最终范围检查以 `c9aa44a..HEAD` 为准时，允许出现本计划文件（它确实是本轮唯一的 `docs/superpowers/plans/` 变更）。

**行号说明：** 全文所有 `file:line` 都是**基线 `c9aa44a` 上的行号**，用于定位，不是编辑后仍成立的行号。同一任务内先做的删除会让后面的行号平移；**以引用的原文片段为准**。

**范围硬约束（执行者必须遵守）：**

1. **只允许修改下列 9 个路径（全部为已有文件，本轮不新增脚本或任何其他文件）：**
   - `include/common/slam_config.h`
   - `src/common/slam_config.cpp`
   - `config/mapping.yaml`
   - `src/mapOptimization.cpp`
   - `include/common/map_artifact_layout.h`
   - `src/common/map_artifact_layout.cpp`
   - `test/test_slam_config.cpp`
   - `test/test_unified_parameter_schema.cpp`
   - `docs/PROJECT_STATUS.md`
2. 禁止修改任何其他文件（包括 `CMakeLists.txt`、launch、`include/lio_sam/utility.hpp`、`src/slam/*`）。唯一例外是本计划文档自身 `docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md`：它在 Phase 0 用 `git add -f` 单独提交（提交信息 `docs: plan shutdown auto-save removal`）后成为受跟踪工件，此后不得再改动。若实测数字与本计划的确定性契约不符，必须停止执行，另起一个 `docs: revise shutdown auto-save removal plan` 提交（禁止 `--amend`）修订本计划后再继续。
3. 禁止 `git add -A` / `git add .`；禁止 `git commit --amend` / `git reset` / `git rebase` / `git checkout` / `git restore` / `git clean` / `git gc` / `git reflog`。只按第 5 条的规则显式暂存具体文件。
4. 不要 push。
5. **`.gitignore` 事实与暂存规则：** 仓库根 `.gitignore` 第 3 行是 `docs/*`（第 4 行 `!docs/PROJECT_STATUS.md` 是例外），第 5 行是 `test/`（整目录忽略，无例外）。因此：
   - 本计划文件当前是被忽略的未跟踪文件（`git check-ignore -v --no-index` 命中 `.gitignore:3:docs/*`，`git ls-files --error-unmatch` 报 `did not match any file(s) known to git`），只能用 `git add -f` 加完整路径暂存；提交后它受版本控制，`git status --short` 才可能显示它。
   - 已跟踪的 `test/*` 一律用 `git add -u` 加显式文件名暂存（`-u` 不受忽略规则影响，且不会误加该目录下的未跟踪文件）。本计划不新增 `test/*` 文件；若确需新增，只能 `git add -f` 加显式文件名。
   - 其余被修改文件（`include/`、`src/`、`config/`、`docs/PROJECT_STATUS.md`）都已跟踪且未被忽略，用普通 `git add` 加完整路径暂存。
   - **所有暂存命令必须显式写完整文件名**，不得用目录或通配符。
6. **静态扫描不得把必要字面量当零匹配，也不得新增扫描脚本。** guard 必须包含 `"map_save.enabled"` 和 `"savePCD"` 各 1 次，`test_unified_parameter_schema.cpp` 的 `ShutdownAutoSaveContract` 测试必须包含 `map_save.enabled` 与 `"savePCD"`。所以“零匹配”只对**生产实现路径**成立；对 guard 文件采用**逐个字面量精确计数（恰好 1 次）**的 allowlist 判定。该判定由 `test_unified_parameter_schema.cpp` 的静态契约（编译进 CTest）和 Task C 的**一次性 `rg`/`grep` 命令加注入自检**共同完成，本轮不新增任何脚本、CMake target 或持久 scanner。
7. **smoke 不使用跨 shell 变量。** 所有路径是固定持久绝对路径，所有判定基于磁盘上的 marker 文件与进程退出码。每个构建/门禁/smoke 执行块都自带 `export HOME=/home/robot`（旧代码用 `std::getenv("HOME") + map_save.directory` 解析导出目录），使基线与最终门禁在相同环境下运行、计数可直接比较。

**固定路径约定（后面所有步骤直接使用，不依赖任何变量）：**

| 用途 | 固定绝对路径 |
| --- | --- |
| 旧二进制证据 | `/home/robot/shutdown_auto_save_evidence/old/lio_sam_hesai_mapOptimization` |
| 新二进制证据 | `/home/robot/shutdown_auto_save_evidence/new/lio_sam_hesai_mapOptimization` |
| smoke 工作目录 | `/home/robot/shutdown_auto_save_evidence/run` |
| smoke 导出目录 | `/home/robot/shutdown_auto_save_smoke` |
| 旧行为导出归档 | `/home/robot/shutdown_auto_save_evidence/old-export` |
| 旧参数阴性 fixture | `/home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml` |
| 改动后参数 fixture（指向 smoke 目录） | `/home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml` |

**旧二进制会写入的产物（阳性对照判据）：** 一定写入的是 `/home/robot/shutdown_auto_save_smoke/trajectory.pcd`、`/home/robot/shutdown_auto_save_smoke/transformations.pcd`、`/home/robot/shutdown_auto_save_smoke/cloudCorner.pcd`、`/home/robot/shutdown_auto_save_smoke/cloudSurf.pcd`、`/home/robot/shutdown_auto_save_smoke/cloudGlobal.pcd` 五个；`cloudGround.pcd` 在空地图下会走 `Ground map is empty, skip saving` 分支而不写，**不作为判据**。

---

## Phase 0：基线构建、门禁、扫描取证与旧行为阳性对照

本阶段**不做任何源码修改**；唯一的仓库写操作是 Task 0.2 用 `git add -f` 提交本计划文件。目的：在 `c9aa44a` 上固定“改动前”事实，并把旧二进制存成后续对照物。

### Task 0.1: 固定基线事实

**Files:**
- 只读：整个仓库

- [ ] **Step 1: 确认 HEAD 与工作区状态**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git log -1 --oneline
git status --short
```

Expected: 第一行是 `c9aa44a docs: design shutdown auto-save removal`；`git status --short` 无输出。

- [ ] **Step 2: 确认忽略规则与计划文件当前未跟踪**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git check-ignore -v --no-index \
  test/test_slam_config.cpp \
  docs/PROJECT_STATUS.md \
  docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
echo "check_ignore_exit=$?"
git ls-files --error-unmatch \
  docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
echo "ls_files_exit=$?"
```

Expected: 必须加 `--no-index`——`git check-ignore` 默认不报告已被跟踪的文件，而 `test/*` 与 `docs/PROJECT_STATUS.md` 都已跟踪，不加该选项会得到空输出（退出码 1），从而误判忽略规则。实际输出应为：

```text
.gitignore:5:test/	test/test_slam_config.cpp
.gitignore:4:!docs/PROJECT_STATUS.md	docs/PROJECT_STATUS.md
.gitignore:3:docs/*	docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
check_ignore_exit=0
error: pathspec 'docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md' did not match any file(s) known to git
Did you forget to 'git add'?
ls_files_exit=1
```

即 `test/` 被第 5 行整目录忽略、`docs/PROJECT_STATUS.md` 被第 4 行反忽略因而不被忽略、本计划文件被第 3 行忽略且尚未跟踪。下一任务用 `git add -f` 把它变成受跟踪工件。

- [ ] **Step 3: 建立固定证据目录**

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_evidence
test ! -e /home/robot/shutdown_auto_save_smoke
mkdir -p /home/robot/shutdown_auto_save_evidence/old
mkdir -p /home/robot/shutdown_auto_save_evidence/new
mkdir -p /home/robot/shutdown_auto_save_evidence/run
ls -d /home/robot/shutdown_auto_save_evidence/old \
      /home/robot/shutdown_auto_save_evidence/new \
      /home/robot/shutdown_auto_save_evidence/run
```

Expected: 三行目录均存在。

### Task 0.2: 把本计划提交为受跟踪工件

**Files:**
- Create: `docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md`（作为 git 跟踪对象；本步不修改文件内容）

- [ ] **Step 1: 强制暂存计划文件**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add -f docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
git status --short
git diff --cached --stat
```

Expected: `git status --short` 恰好一行 `A  docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md`；`git diff --cached --stat` 只有这 1 个文件。不得出现其他路径。

- [ ] **Step 2: 提交计划**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git commit -m "docs: plan shutdown auto-save removal"
git log -2 --oneline
```

Expected: 提交成功；`git log -2 --oneline` 第一行是 `docs: plan shutdown auto-save removal`，第二行是 `c9aa44a docs: design shutdown auto-save removal`。

- [ ] **Step 3: 确认计划已成为受跟踪文件**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git ls-files --error-unmatch \
  docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
echo "plan_tracked_exit=$?"
git status --short
```

Expected: `git ls-files` 打印该路径且 `plan_tracked_exit=0`；`git status --short` 无输出（工作区干净，HEAD 为计划提交）。这样 Task E 在 `docs/PROJECT_STATUS.md` 第 10 节写的相对链接 `superpowers/plans/2026-09-15-remove-shutdown-auto-save.md` 指向真实受跟踪文件，Task E Step 7 会再校验一次。

### Task 0.3: 基线构建与约定门禁

**Files:**
- 只读：`CMakeLists.txt`、`test/`

- [ ] **Step 1: 基线构建**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1
```

Expected: 退出码 0，末尾 `[100%] Built target ...`。若该构建树不存在，改用下面的备选命令：

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cd /home/robot/slam_ws
colcon build --symlink-install --packages-select lio_sam_hesai \
  --executor sequential --parallel-workers 1 \
  --cmake-args -DBUILD_TESTING=ON
```

- [ ] **Step 2: 运行约定 CTest 门禁并记录真实计数**

Run:

```bash
export HOME=/home/robot
set -o pipefail
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure -j1 \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$' \
  | tee /home/robot/shutdown_auto_save_evidence/run/baseline-ctest.txt
```

Expected: 末尾 `100% tests passed, 0 tests failed out of 44`。44 是当前约定门禁的契约值（`docs/PROJECT_STATUS.md` 9.9 记录 44/44）。**以终端实际输出为准**：若实际不是 44/44，停止后续步骤并记录差异，先按第 0 节第 2 条修订本计划中的 44 与 Task E 的确定性模板后再继续；不得把未修订的数字写进 `docs/PROJECT_STATUS.md`。

- [ ] **Step 3: 记录基线静态扫描事实（允许有匹配，这是“改动前”证据）**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
rg -n --no-heading \
  'save_map_enabled_|map_save\.enabled|"savePCD"|Saved(Corner|Surface|Global|Ground)Cloud|kSaved(Corner|Surface|Global|Ground)CloudName|cloud(Corner|Surf|Global|Ground)\.pcd' \
  include src config launch test \
  | tee /home/robot/shutdown_auto_save_evidence/run/baseline-tokens.txt
wc -l /home/robot/shutdown_auto_save_evidence/run/baseline-tokens.txt
```

Expected: 有匹配（基线必然如此，实测 28 行）。必须出现的关键行包括
`src/mapOptimization.cpp:260:    bool save_map_enabled_ = false;`、
`src/common/slam_config.cpp:512:  map_save.enabled = ReadParam<bool>(node, "map_save.enabled",`、
`include/common/map_artifact_layout.h:37:  static constexpr const char* kSavedCornerCloudName = "cloudCorner.pcd";`、
`test/test_unified_parameter_schema.cpp:117:  {"savePCD", "map_save.enabled", kMappingSurfaces},`。
注意 `src/common/slam_config.cpp` 中的 `"savePCDDirectory"`、`"savePCDResolution"`、`pcl::io::savePCDFileBinary` **不会**被该表达式匹配（它们不是带引号的独立 `"savePCD"`）。

- [ ] **Step 4: 确认新 guard 在基线上不存在**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
rg -n 'RejectRemovedMappingParameters|was removed with Ctrl\+C' src/common/slam_config.cpp
```

Expected: 无输出，退出码 1。这两条正是后续 Task B 要加入的 fail-closed guard 标识。

### Task 0.4: 旧行为阳性对照 smoke（旧二进制 + 旧参数）

**关键理解：** `mapOptimization` 主线程 `exec.spin()` 在 Ctrl+C 后返回，`rclcpp::shutdown()` 把 `rclcpp::ok()` 置为 false，`visualizeGlobalMapThread()` 因此退出 `while (rclcpp::ok())` 循环，**然后**执行 `if (save_map_enabled_ == false) return;` 之后的导出代码。所以不播 bag、不启 rviz、没有 keyframe 也会写 `trajectory.pcd`、`transformations.pcd`、`cloudCorner.pcd`、`cloudSurf.pcd`、`cloudGlobal.pcd`。`publishGlobalMap()` 在没有订阅者或没有 keyframe 时提前返回，空跑 12 秒是安全的。

**为什么不用 `launch/mapping.launch.py`：** 该 launch 除了 `lio_sam_hesai_mapOptimization` 还会启动两个 static TF、`imuPreintegration`、`imageProjection`、`featureExtraction` 和 `rviz2`（`launch/mapping.launch.py` 末尾），既引入 rviz 又让 SIGINT 的落点变成 launch 进程，无法把退出码归因到被观察的节点。而 `visualizeGlobalMapThread()` 只在 `MO->MappingMode()` 为真时启动（基线 `src/mapOptimization.cpp:2759`），单独 `ros2 run` 该节点就足以走出完整关闭路径，且 PID/退出码可以直接管理。因此本计划的 smoke 一律直接用 `ros2 run`。

**Files:**
- 只写证据目录，**不修改仓库任何文件**

- [ ] **Step 1: 生成旧参数 fixture（`enabled: true` + 固定导出目录）**

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_smoke
test ! -e /home/robot/shutdown_auto_save_evidence/old-export
cp /home/robot/slam_ws/src/lio_sam_hesai/config/mapping.yaml \
   /home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml
sed -i \
  -e 's#^      enabled: false.*#      enabled: true#' \
  -e 's#^      directory: .*#      directory: "/shutdown_auto_save_smoke/"#' \
  /home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml
grep -n '^      enabled:\|^      directory:' \
  /home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml
test ! -e /home/robot/shutdown_auto_save_smoke
```

Expected: 输出恰好两行 —— `      enabled: true` 和
`      directory: "/shutdown_auto_save_smoke/"`；最后一条 `test ! -e` 退出码 0（目录尚不存在）。

- [ ] **Step 2: 备份基线旧二进制**

Run:

```bash
cp /home/robot/slam_ws/build/lio_sam_hesai/lio_sam_hesai_mapOptimization \
   /home/robot/shutdown_auto_save_evidence/old/lio_sam_hesai_mapOptimization
ls -l /home/robot/shutdown_auto_save_evidence/old/lio_sam_hesai_mapOptimization
```

Expected: 文件存在且大小非 0。这是 `c9aa44a` 上编出的旧二进制，后续 Task A/B 重编不会覆盖它。

同时注意：`ros2 run` 解析到的是 `install/lio_sam_hesai/lib/lio_sam_hesai/lio_sam_hesai_mapOptimization`，它是 `--symlink-install` 建立的符号链接，指向 `build/lio_sam_hesai/lio_sam_hesai_mapOptimization`。所以在 Phase 0 期间（重编 `mapOptimization` 之前）它就是基线旧二进制；这份 `cp` 备份是防止后续重编覆盖的保险。

- [ ] **Step 3: 旧二进制阳性对照（SIGINT 后必须写文件）**

**为什么不能只 `kill -INT $!`：** `ros2 run` 的实现是 `subprocess.Popen(cmd)` + `communicate()`（见 `/opt/ros/humble/lib/python3.10/site-packages/ros2run/api/__init__.py`），它**不转发信号**；只有终端把 Ctrl+C 发给整个前台进程组时子进程才收得到。所以必须先用 `pgrep -P` 找到真正的节点子进程，再把 SIGINT 发给它，并用有界轮询避免 `wait` 永久挂起。

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 run lio_sam_hesai lio_sam_hesai_mapOptimization --ros-args \
  --params-file /home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml \
  > /home/robot/shutdown_auto_save_evidence/run/old-positive.log 2>&1 &
wrapper_pid=$!
echo "$wrapper_pid" > /home/robot/shutdown_auto_save_evidence/run/old-positive.pid
sleep 12
node_pid=$(pgrep -P "$wrapper_pid" -f lio_sam_hesai_mapOptimization | head -n 1)
echo "node_pid=$node_pid"
test -n "$node_pid"
kill -INT "$node_pid"
for _ in $(seq 1 40); do
  kill -0 "$wrapper_pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$wrapper_pid" 2>/dev/null; then
  echo "wrapper_still_running=1"
  kill -KILL "$node_pid" 2>/dev/null || true
  kill -KILL "$wrapper_pid" 2>/dev/null || true
else
  echo "wrapper_still_running=0"
fi
wait "$wrapper_pid"
echo "old_exit=$?"
```

`export HOME=/home/robot` 是必须的：旧代码把导出目录解析为 `std::getenv("HOME") + map_save.directory`，只有 HOME 确定时 `/home/robot/shutdown_auto_save_smoke/` 这个固定路径才成立。

Expected: `node_pid=` 是一个非空数字（`test -n` 退出码 0）；`wrapper_still_running=0`（节点在 40 秒内自行退出）；`old_exit=0`；日志中出现 `Saving map to pcd files ...` 与 `Saving map to pcd files completed`。

若 `node_pid` 为空：说明 `ros2 run` 的进程模型与预期不同，**停止执行并记录该事实**，不要继续用未验证的进程控制方式做 smoke。

- [ ] **Step 4: 断言旧二进制确实写入了历史产物**

Run:

```bash
test -f /home/robot/shutdown_auto_save_smoke/trajectory.pcd
test -f /home/robot/shutdown_auto_save_smoke/transformations.pcd
test -f /home/robot/shutdown_auto_save_smoke/cloudCorner.pcd
test -f /home/robot/shutdown_auto_save_smoke/cloudSurf.pcd
test -f /home/robot/shutdown_auto_save_smoke/cloudGlobal.pcd
ls -l /home/robot/shutdown_auto_save_smoke/
touch /home/robot/shutdown_auto_save_evidence/run/old-positive.marker
```

Expected: 五条 `test -f` 全部退出码 0；`ls` 列出 5 个 `.pcd`（`cloudGround.pcd` 允许不存在，空地图会打印 `Ground map is empty, skip saving`）；marker 文件创建成功。

- [ ] **Step 5: 归档旧导出目录，为 Task A 的“新二进制零写入”对照准备**

Run:

```bash
mv /home/robot/shutdown_auto_save_smoke \
   /home/robot/shutdown_auto_save_evidence/old-export
test ! -e /home/robot/shutdown_auto_save_smoke
test -f /home/robot/shutdown_auto_save_evidence/old-export/cloudGlobal.pcd
```

Expected: 退出码 0。旧输出以可恢复方式归档到 `old-export/`，阳性对照证据由
`old-positive.log`、`old-positive.marker` 与该目录共同保存；smoke 目标目录回到不存在状态。

- [ ] **Step 6: Phase 0 交接记录（本阶段唯一提交是 Task 0.2 的计划提交）**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git status --short
git log -2 --oneline
```

Expected: `git status --short` 无输出（除 Task 0.2 的计划提交外，Phase 0 只写了 `/home/robot/...` 下的证据文件）；`git log -2 --oneline` 顶部两行是 `docs: plan shutdown auto-save removal`、`c9aa44a docs: design shutdown auto-save removal`。

---

## Task A：删除运行期关闭路径与 `Saved*Cloud()` API

**Files:**
- Modify: `test/test_unified_parameter_schema.cpp`（新增一个 absence-contract 测试）
- Modify: `src/mapOptimization.cpp:258-260`（成员）、`src/mapOptimization.cpp:297-307`（构造函数）、`src/mapOptimization.cpp:948-1024`（`visualizeGlobalMapThread()` 尾部）
- Modify: `include/common/map_artifact_layout.h:34-40,77-80`
- Modify: `src/common/map_artifact_layout.cpp:113-129`

**本任务结束时必须满足：** `config_.mapping.map_save.enabled` 字段**仍然存在**（由 Task B 删除），但已无任何运行期读者；`lio_sam_hesai_mapOptimization` 可独立编译；新增 absence-contract 测试通过。

- [ ] **Step 1（RED）: 写失败的 absence-contract 测试**

在 `test/test_unified_parameter_schema.cpp` 末尾（`YamlScannerStopsOutsideRosParameters` 测试的最后一个 `}` 之后）追加：

```cpp

TEST(UnifiedParameterSchema, ShutdownAutoSaveImplementationIsAbsent) {
  const std::vector<std::pair<std::string, std::string>> forbidden = {
    {"src/mapOptimization.cpp", "save_map_enabled_"},
    {"src/mapOptimization.cpp", "SavedCornerCloud"},
    {"src/mapOptimization.cpp", "SavedSurfaceCloud"},
    {"src/mapOptimization.cpp", "SavedGlobalCloud"},
    {"src/mapOptimization.cpp", "SavedGroundCloud"},
    {"include/common/map_artifact_layout.h", "kSavedCornerCloudName"},
    {"include/common/map_artifact_layout.h", "kSavedSurfaceCloudName"},
    {"include/common/map_artifact_layout.h", "kSavedGlobalCloudName"},
    {"include/common/map_artifact_layout.h", "kSavedGroundCloudName"},
    {"include/common/map_artifact_layout.h", "SavedCornerCloud"},
    {"include/common/map_artifact_layout.h", "SavedSurfaceCloud"},
    {"include/common/map_artifact_layout.h", "SavedGlobalCloud"},
    {"include/common/map_artifact_layout.h", "SavedGroundCloud"},
    {"include/common/map_artifact_layout.h", "\"cloudCorner.pcd\""},
    {"include/common/map_artifact_layout.h", "\"cloudSurf.pcd\""},
    {"include/common/map_artifact_layout.h", "\"cloudGlobal.pcd\""},
    {"include/common/map_artifact_layout.h", "\"cloudGround.pcd\""},
    {"src/common/map_artifact_layout.cpp",
     "MapArtifactLayout::SavedCornerCloud"},
    {"src/common/map_artifact_layout.cpp",
     "MapArtifactLayout::SavedSurfaceCloud"},
    {"src/common/map_artifact_layout.cpp",
     "MapArtifactLayout::SavedGlobalCloud"},
    {"src/common/map_artifact_layout.cpp",
     "MapArtifactLayout::SavedGroundCloud"},
  };

  for (const auto & [file, token] : forbidden) {
    SCOPED_TRACE(file + ": " + token);
    EXPECT_EQ(ReadProjectFile(file).find(token), std::string::npos);
  }
}
```

**不要**为本任务改动 include 区：该测试只用 `std::string`、`std::vector`、`std::pair`、`gtest`，现有 include 已足够。（Task B 新增的计数 helper 自带所需类型，同样不需要新 include。）

- [ ] **Step 2（RED）: 编译并确认测试失败**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_unified_parameter_schema -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^test_unified_parameter_schema$' --output-on-failure
```

Expected: 构建成功；测试失败，失败信息里出现 `src/mapOptimization.cpp: save_map_enabled_`、`include/common/map_artifact_layout.h: kSavedCornerCloudName` 等 trace。

- [ ] **Step 3（GREEN）: 删除运行期标志与其构造函数赋值**

在 `src/mapOptimization.cpp` 中把 258-260 行的成员块：

```cpp
    // Runtime overrides of two configured flags: see the constructor.
    bool loop_closure_enabled_ = false;
    bool save_map_enabled_ = false;
```

替换为：

```cpp
    // Runtime override of the configured loop-closure flag: the node turns it
    // off for itself when it is not running in mapping mode.
    bool loop_closure_enabled_ = false;
```

再把 297-307 行的构造函数逻辑：

```cpp
        // Runtime overrides rather than configuration: the node turns these
        // off for itself when it is not running in mapping mode, so they live
        // here instead of in the immutable config_.
        loop_closure_enabled_ = config_.mapping.loop_closure.enable;
        save_map_enabled_ = config_.mapping.map_save.enabled;
        if (!config_.mapping.system.mapping_mode)
        {
            loop_closure_enabled_ = false;
            save_map_enabled_ = false;
            RCLCPP_WARN(get_logger(), "mappingMode=false: disable loop closure and map saving");
        }
```

替换为：

```cpp
        // Runtime override rather than configuration: the node turns this off
        // for itself when it is not running in mapping mode, so it lives here
        // instead of in the immutable config_.
        loop_closure_enabled_ = config_.mapping.loop_closure.enable;
        if (!config_.mapping.system.mapping_mode)
        {
            loop_closure_enabled_ = false;
            RCLCPP_WARN(get_logger(), "mappingMode=false: disable loop closure");
        }
```

**不要**改动 `if (config_.mapping.system.mapping_mode) { ... create_service ... }`（328 行起）与 `RCLCPP_WARN(get_logger(), "mappingMode=false: save_map service disabled");`（569 行）——显式保存服务必须保留。

- [ ] **Step 4（GREEN）: 让可视化线程退出即结束**

在 `src/mapOptimization.cpp` 中把 `visualizeGlobalMapThread()` 整体替换为（保留发布循环，删除循环之后的全部导出语句，含 `save_map_enabled_ == false` 早退、`rm -r`/`mkdir`、`MapArtifactLayout shutdown_layout`、`buildFullGlobalGroundMap` 相关的导出与两条 `std::cout`）：

```cpp
    void visualizeGlobalMapThread()
    {
        rclcpp::Rate rate(1.0);
        while (rclcpp::ok()){
            rate.sleep();
            if (!config_.mapping.system.mapping_mode)
                continue;
            publishGlobalMap();
            publishGlobalGroundMap();
        }
    }
```

**保留** `shiftSavedMapCloudZ()`（`src/mapOptimization.cpp:1027`）、`flattenGroundCloudForSave()`、`appendGlobalGroundGridPatches()` 和 `saveGlobalGroundMap()`：显式保存服务仍调用它们。

**关于被删掉的 1020-1021 行 `downSizeFilterCorner/Surf.setLeafSize(...)` 复原语句：** 它们可以一起删除。该尾部代码此前把共享降采样器的 leaf size 改成 `map_save.resolution`，再在结尾复原；删除后不再有这处改动，因此不需要复原。唯一读这两个过滤器的地方是 `publishGlobalMap()`（786-792 行）与建图主流程（1732-1772 行），而它们都在 `visualizeGlobalMapThread()` 的 `while` 循环**之前或循环之内**执行，尾部代码只在循环退出后运行一次，不存在与主流程交错的窗口。显式保存服务自己会在 478-485 行改用 `req->resolution`，并在 558-559 行复原为 `voxel_filter.mapping_*_leaf_size`，因此服务路径也不依赖这段被删代码。

- [ ] **Step 5（GREEN）: 删除仅供关闭路径使用的产物 API**

在 `include/common/map_artifact_layout.h` 中删除 34-40 行整块：

```cpp
  // Written only by the shutdown auto-save. Nothing reads these back, but P2.2
  // does not rename on-disk artifacts, so they keep their historical names and
  // still come from here instead of being concatenated at the call site.
  static constexpr const char* kSavedCornerCloudName = "cloudCorner.pcd";
  static constexpr const char* kSavedSurfaceCloudName = "cloudSurf.pcd";
  static constexpr const char* kSavedGlobalCloudName = "cloudGlobal.pcd";
  static constexpr const char* kSavedGroundCloudName = "cloudGround.pcd";
```

并删除 77-80 行这四条声明：

```cpp
  [[nodiscard]] std::filesystem::path SavedCornerCloud() const;
  [[nodiscard]] std::filesystem::path SavedSurfaceCloud() const;
  [[nodiscard]] std::filesystem::path SavedGlobalCloud() const;
  [[nodiscard]] std::filesystem::path SavedGroundCloud() const;
```

在 `src/common/map_artifact_layout.cpp` 中删除 `MapArtifactLayout::SavedCornerCloud()`、`MapArtifactLayout::SavedSurfaceCloud()`、`MapArtifactLayout::SavedGlobalCloud()`、`MapArtifactLayout::SavedGroundCloud()` 四个完整函数定义（当前 115-129 行区间）。

**保留** `GlobalMap()`、`GroundMap()`、`ScanContextDb()`、`Trajectory()`、`Transformations()`、`CornerMap()`、`SurfaceMap()`、`TiledRoot()`、`TiledManifest()`、`RootDirectoryOf()`、`FromGlobalMap()`、`FromTiledRoot()` 及头文件顶部注释里的 `GlobalMap.pcd`/`GroundMap.pcd`/`CornerMap.pcd`/`SurfMap.pcd` 布局说明（它们属于显式服务契约）。

- [ ] **Step 6（GREEN）: 确认 absence-contract 测试通过**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_unified_parameter_schema -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^test_unified_parameter_schema$' --output-on-failure
```

Expected: `100% tests passed`，`ShutdownAutoSaveImplementationIsAbsent` 通过。

- [ ] **Step 7（GREEN）: 确认 mapOptimization 任务内可独立编译**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target lio_sam_hesai_mapOptimization -- -j1
```

Expected: 退出码 0，`[100%] Built target lio_sam_hesai_mapOptimization`。

再确认运行期已无该字段的读者（此时字段本身仍在 `slam_config.h` 里，等 Task B 删除）：

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
rg -n 'map_save\.enabled|save_map_enabled_' src/mapOptimization.cpp
```

Expected: 无输出，退出码 1。若仍有输出，说明 Step 3 漏删了某处读取。

- [ ] **Step 8（REFACTOR 后的回归）: 跑与产物布局相关的既有测试**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_map_artifact_layout test_slam_config -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_map_artifact_layout|test_slam_config)$' --output-on-failure
```

Expected: 2/2 通过。此时 `test_slam_config` 尚未新增 removed-parameter 测试，应保持全绿。

- [ ] **Step 9: 暂存并提交 Task A**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add src/mapOptimization.cpp include/common/map_artifact_layout.h \
  src/common/map_artifact_layout.cpp
git add -u test/test_unified_parameter_schema.cpp
git status --short
git diff --cached --stat
git commit -m "refactor(mapping): delete shutdown auto-save runtime path"
```

Expected: `git status --short` 只显示这 4 个文件（`M`）；`--stat` 恰好 4 个文件；提交成功。

---

## Task B：删除配置字段/parser/YAML/别名，并加 raw override fail-closed guard

**Files:**
- Modify: `include/common/slam_config.h:100-101`
- Modify: `src/common/slam_config.cpp:22-24`（`Reject` 之后新增 guard）、`src/common/slam_config.cpp:511-514`（删除读取）、`src/common/slam_config.cpp:1178`（`LoadSlamConfig` 入口调用 guard）
- Modify: `config/mapping.yaml:89-91`
- Modify: `test/test_slam_config.cpp`（新增 removed-parameter 测试）
- Modify: `test/test_unified_parameter_schema.cpp:117`（删除旧别名契约）＋新增反向契约测试

- [ ] **Step 1（RED）: 写失败的配置行为测试**

在 `test/test_slam_config.cpp` 的 `MappingRejectsNonPositiveOrNonFiniteTileSize` 测试（结束于 295 行的 `}`）之后追加：

```cpp

TEST(SlamConfig, RejectsRemovedShutdownAutoSaveParameters) {
  // Ctrl+C auto-save was deleted. Both the canonical key and its flat alias
  // must now fail closed at startup instead of being silently accepted, and
  // the error must name the replacement interface.
  for (const std::string key : {"map_save.enabled", "savePCD"}) {
    SCOPED_TRACE(key);
    try {
      LoadMapping({rclcpp::Parameter(key, true)});
      FAIL() << "accepted removed shutdown auto-save parameter";
    } catch (const std::invalid_argument& error) {
      const std::string message = error.what();
      EXPECT_NE(message.find(key), std::string::npos);
      EXPECT_NE(message.find("/lio_sam/save_map"), std::string::npos);
    }
  }
}

```

- [ ] **Step 2（RED）: 写失败的 YAML/launch 反向契约与字面量 allowlist 测试**

在 `test/test_unified_parameter_schema.cpp` 中先删除 `kParameterContracts` 表里的这一行（当前 117 行）：

```cpp
  {"savePCD", "map_save.enabled", kMappingSurfaces},
```

`generateScDb`、`savePCDDirectory`、`savePCDResolution`、`saveGroundPCD` 等别名**不得删除**：它们仍然配置显式保存服务。

然后在文件末尾（Task A 新增测试之后）追加。**注意 `CountOccurrences` 不能用 `std::count`**：`std::count(first, last, value)` 是逐元素比较，把 `std::string` 当 `value` 只会统计“单字符等于该字符串”的假匹配，不会统计子串出现次数，必须用显式的 `find` 循环：

```cpp

namespace {

std::size_t CountOccurrences(const std::string & text,
                             const std::string & needle) {
  std::size_t count = 0;
  for (std::size_t pos = text.find(needle); pos != std::string::npos;
       pos = text.find(needle, pos + needle.size())) {
    ++count;
  }
  return count;
}

}  // namespace

TEST(UnifiedParameterSchema, ShutdownAutoSaveContract) {
  const std::string mapping_yaml = ReadProjectFile("config/mapping.yaml");
  const std::string mapping_launch = StripPythonComments(
    ReadProjectFile("launch/mapping.launch.py"));
  const std::string guard_file =
    ReadProjectFile("src/common/slam_config.cpp");

  // Reverse contract: the removed keys must not come back through the YAML or
  // any launch override. "enabled:" is forbidden because it only ever existed
  // under map_save, which now has exactly one export path.
  EXPECT_EQ(mapping_yaml.find("\"savePCD\""), std::string::npos);
  EXPECT_EQ(mapping_yaml.find("enabled:"), std::string::npos);
  EXPECT_EQ(mapping_yaml.find("map_save.enabled"), std::string::npos);
  EXPECT_FALSE(HasQuotedString(mapping_launch, "savePCD"));
  EXPECT_FALSE(HasQuotedString(mapping_launch, "map_save.enabled"));

  // The fail-closed guard is the only place allowed to keep the removed
  // literals, and it must keep exactly one copy of each: a naive zero-match
  // scan would either miss a dropped guard or be forced to ignore real
  // re-introductions.
  const std::vector<std::string> guard_literals = {
    "\"map_save.enabled\"", "\"savePCD\"",
  };
  for (const auto & literal : guard_literals) {
    SCOPED_TRACE(literal);
    EXPECT_EQ(CountOccurrences(guard_file, literal), std::size_t{1});
  }

  // The historical artifact names must not survive anywhere in the tree.
  const std::vector<std::string> removed_artifacts = {
    "cloudCorner.pcd", "cloudSurf.pcd", "cloudGlobal.pcd", "cloudGround.pcd",
  };
  for (const auto & artifact : removed_artifacts) {
    SCOPED_TRACE(artifact);
    EXPECT_EQ(mapping_yaml.find(artifact), std::string::npos);
  }
}
```

**关于 `guard_literals` 计数的语义（重要）：** 该断言在 Step 2 结束时**已经成立**，不是 RED 信号。基线 `src/common/slam_config.cpp` 的第 512-513 行恰好各含一次 `"map_save.enabled"` 与 `"savePCD"`：

```cpp
  map_save.enabled = ReadParam<bool>(node, "map_save.enabled",
                                     std::string("savePCD"),
                                     map_save.enabled, warnings);
```

Step 5 会删掉这三行，Step 4 会把同样两个字面量放进 guard——计数保持 `1 → 1`。这正是本计划要的 allowlist 语义：**零匹配只对生产实现路径成立，guard 文件必须保留必要字面量，且必须恰好一份**。若 Step 5 删了读取语句而 Step 4 忘了写 guard，这个断言就会变成 `1 → 0` 并失败；若有人把参数重新声明回来，计数变成 2 也会失败。Step 2 的 RED 信号来自上面的 YAML 反向断言（`enabled:` 与 `"savePCD"` 仍在 `config/mapping.yaml` 中）。

**注意：** 该测试必须在 Task B 的 Step 3/4/5/6 完成后才变绿；Step 2 结束时它必然因 YAML 断言失败。

- [ ] **Step 3（RED）: 编译并确认两个测试都失败**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_slam_config test_unified_parameter_schema -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_slam_config|test_unified_parameter_schema)$' --output-on-failure
```

Expected: 构建成功；`test_slam_config` 在 `RejectsRemovedShutdownAutoSaveParameters` 失败（两条 key 当前仍被接受，`FAIL() << "accepted removed shutdown auto-save parameter"`）；`test_unified_parameter_schema` 在 `ShutdownAutoSaveContract` 失败，失败点是 `config/mapping.yaml` 仍含 `find("enabled:")` 与 `find("\"savePCD\"")` 两处非 `npos` 命中。此时 guard 字面量计数断言应当**通过**（基线第 512-513 行已各含一次），不要把它当成 RED 信号。

- [ ] **Step 4（GREEN）: 加入 fail-closed raw override guard**

在 `src/common/slam_config.cpp` 匿名命名空间的 `Reject`（22-24 行）之后加入：

```cpp
// Rejects parameters that were removed with the Ctrl+C auto-save path. The
// keys are deliberately not declared any more, so the guard reads the raw
// override map: a declared-key check would silently accept them and a
// typo'd config would look like a working one.
void RejectRemovedMappingParameters(rclcpp::Node& node) {
  const auto overrides =
      node.get_node_parameters_interface()->get_parameter_overrides();
  for (const std::string& name : {"map_save.enabled", "savePCD"}) {
    if (overrides.count(name) != 0U) {
      Reject(name,
             "was removed with Ctrl+C automatic map saving; use "
             "/lio_sam/save_map instead");
    }
  }
}
```

在 `LoadSlamConfig`（当前 1178 行）里把：

```cpp
SlamConfig LoadSlamConfig(rclcpp::Node& node, SlamMode mode) {
  SlamConfig config;
  config.mode = mode;
  LegacyWarningSet warnings;

  LoadFrames(node, mode, config, warnings);
```

改为：

```cpp
SlamConfig LoadSlamConfig(rclcpp::Node& node, SlamMode mode) {
  SlamConfig config;
  config.mode = mode;
  LegacyWarningSet warnings;

  if (mode == SlamMode::kMapping) {
    RejectRemovedMappingParameters(node);
  }

  LoadFrames(node, mode, config, warnings);
```

guard 必须在任何 `Load*` 之前执行，否则同一次调用里的其他参数校验可能先抛出无关错误。

- [ ] **Step 5（GREEN）: 删除字段与读取语句**

在 `include/common/slam_config.h` 的 `struct MapSaveConfig` 中删除：

```cpp
  bool enabled{false};
```

在 `src/common/slam_config.cpp` 中删除：

```cpp
  map_save.enabled = ReadParam<bool>(node, "map_save.enabled",
                                     std::string("savePCD"),
                                     map_save.enabled, warnings);
```

- [ ] **Step 6（GREEN）: 删除 YAML leaf 并更新注释**

把 `config/mapping.yaml` 的 89-91 行：

```yaml
    # CTRL+C地图与点云导出
    map_save:
      enabled: false                                     # 程序退出时是否自动保存全局地图点云
```

替换为：

```yaml
    # 显式地图与点云导出（仅通过 /lio_sam/save_map 服务触发）
    map_save:
```

**保留** `save_tiled_map`、`generate_scan_context_db`、`directory`、`resolution`、`ground_resolution`、`save_ground_map`、`ground_directory`、`ground_global_directory`、`ground_final_use_odometry`、`ground_global_leaf_size`、`align_ground_to_zero`、`ground_z_offset` 及其注释：显式服务全部消费它们。

- [ ] **Step 7（GREEN）: 确认两个测试都通过**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai \
  --target test_slam_config test_unified_parameter_schema \
  lio_sam_hesai_mapOptimization -- -j1
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_slam_config|test_unified_parameter_schema)$' --output-on-failure
```

Expected: 2/2 通过；`RejectsRemovedShutdownAutoSaveParameters` 证明两条 key 都 fail-closed 且错误串含 `/lio_sam/save_map`；`ShutdownAutoSaveContract` 证明 YAML/launch 无反引入且 guard 字面量各恰好 1 次。

- [ ] **Step 8: 暂存并提交 Task B**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add include/common/slam_config.h src/common/slam_config.cpp \
  config/mapping.yaml
git add -u test/test_slam_config.cpp test/test_unified_parameter_schema.cpp
git status --short
git diff --cached --stat
git commit -m "fix(config): reject removed shutdown auto-save parameters"
```

Expected: `git status --short` 只显示这 5 个文件；提交成功。

---

## Phase 1：端到端验证、一次性静态扫描与 PROJECT_STATUS 收口

### Task C: 一次性静态 allowlist 扫描与注入自检（不新增文件、不产生提交）

**Files:**
- 只读：`include/`、`src/`、`config/`、`launch/`
- 只写：`/home/robot/shutdown_auto_save_evidence/run/`

**扫描设计（避免“把必要字面量当零匹配”）：** 生产路径（`include/ src/ config/ launch/`）的历史 token 必须为 0；guard 文件 `src/common/slam_config.cpp` 的 `map_save.enabled` 与 `savePCD` 字面量必须**各自恰好 1 次**；`test/` 目录不在扫描范围内（新的静态契约测试按定义包含这些字面量）。同一语义已由 `test_unified_parameter_schema.cpp` 的 `ShutdownAutoSaveImplementationIsAbsent` 与 `ShutdownAutoSaveContract` 固化，本任务只是**一次性** `rg` 交叉验证加注入自检，**不新增脚本、不新增文件、不产生提交**。

- [ ] **Step 1: 三路 `rg` 交叉验证 + guard 精确计数**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
echo "--- A. production implementation tokens (expect no output) ---"
rg -n --no-heading \
  'save_map_enabled_|Saved(Corner|Surface|Global|Ground)Cloud|kSaved(Corner|Surface|Global|Ground)CloudName' \
  include src config launch
echo "exit_A=$?"
echo "--- B. historical artifact names outside test/ (expect no output) ---"
rg -n --no-heading 'cloud(Corner|Surf|Global|Ground)\.pcd' \
  include src config launch
echo "exit_B=$?"
echo "--- C. removed parameters outside the guard (expect no output) ---"
rg -n --no-heading \
  'map_save\.enabled|"savePCD"' include src config launch \
  -g '!src/common/slam_config.cpp'
echo "exit_C=$?"
echo "--- D. the guard must keep exactly one copy of each (expect 1) ---"
rg -n --no-heading 'map_save\.enabled|"savePCD"' src/common/slam_config.cpp \
  | tee /home/robot/shutdown_auto_save_evidence/run/guard-literals.txt
wc -l /home/robot/shutdown_auto_save_evidence/run/guard-literals.txt
rg -o --no-filename 'map_save\.enabled|"savePCD"' src/common/slam_config.cpp \
  | sort | uniq -c
```

Expected: A/B/C 三段均无输出，且 `exit_A=1`、`exit_B=1`、`exit_C=1`（`rg` 无匹配时返回 1）。`-g '!src/common/slam_config.cpp'` 对显式给出的文件路径确实生效：基线在同一命令下只命中 `src/mapOptimization.cpp:301` 的旧读者，guard 文件里的 `map_save.enabled` / `savePCD` 不出现；Task A/B 删完旧读者后 A/B/C 必然全空。D 段 `wc -l` 输出 `1`（只有 guard 那一行），且 `rg -o | sort | uniq -c` 恰好两行、计数各为 1：

```text
      1 "savePCD"
      1 map_save.enabled
```

这正是“guard 保留必要字面量”的判定：**不是零匹配，而是每个字面量恰好 1 次**。基线同一条命令会多出 `src/mapOptimization.cpp:301` 与 `src/common/slam_config.cpp` 512-514 行的读取语句，改完后只剩 guard 的 1 行。

- [ ] **Step 2: 在证据副本上做注入自检，证明扫描真的会失败**

本步只修改证据目录里的 YAML 副本，不触碰仓库文件。向副本追加两个已知坏 token，
验证第 1 步的扫描式样会命中，再确认仓库 `config/mapping.yaml` 仍与备份逐字节相同。

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
cp config/mapping.yaml /home/robot/shutdown_auto_save_evidence/run/mapping.yaml.bak
cp config/mapping.yaml /home/robot/shutdown_auto_save_evidence/run/mapping.yaml.injected
printf '\n    reintroduced: map_save.enabled\n    old_name: cloudCorner.pcd\n' \
  >> /home/robot/shutdown_auto_save_evidence/run/mapping.yaml.injected
rg -n --no-heading \
  'map_save\.enabled|cloud(Corner|Surf|Global|Ground)\.pcd' \
  /home/robot/shutdown_auto_save_evidence/run/mapping.yaml.injected
echo "injected_scan_exit=$?"
rg -n --no-heading \
  'map_save\.enabled|cloud(Corner|Surf|Global|Ground)\.pcd' config/mapping.yaml
echo "repository_scan_exit=$?"
cmp config/mapping.yaml /home/robot/shutdown_auto_save_evidence/run/mapping.yaml.bak
echo "repository_cmp_exit=$?"
git status --short
git diff --stat config/mapping.yaml
```

Expected: 注入副本的 `rg` 恰好输出 2 行，分别含 `reintroduced: map_save.enabled` 与
`old_name: cloudCorner.pcd`（行号取决于副本末尾），`injected_scan_exit=0`；仓库文件扫描无输出、
`repository_scan_exit=1`；`repository_cmp_exit=0`；`git status --short` 与
`git diff --stat` 均无输出。判定标准是“副本注入后必须命中，仓库文件始终无匹配且与备份相同”；
否则先修正扫描命令再继续。

### Task D: 二进制对照 smoke 与 `/lio_sam/save_map` 服务接口 smoke

**Files:**
- 只读：`config/mapping.yaml`、`src/mapOptimization.cpp:565`
- 只写：`/home/robot/shutdown_auto_save_evidence/**`、`/home/robot/shutdown_auto_save_smoke/**`

**判定矩阵：**

| 场景 | 二进制 | 参数 | 期望 |
| --- | --- | --- | --- |
| 旧阳性（Phase 0 已完成） | 旧 | `enabled: true` | 写 5 个历史 pcd |
| 新阳性 | 新 | 基线 `mapping.yaml` | 退出码 0，零写入 |
| 新 fail-closed | 新 | `legacy-enabled.yaml` | 启动即非零退出，日志含迁移提示，零写入 |
| 服务接口 smoke | 新 | 基线 `mapping.yaml` | `/lio_sam/save_map` 已注册且类型为 `lio_sam_hesai/srv/SaveMap`（只验证注册/类型，不调用导出） |

- [ ] **Step 1: 备份新二进制**

Run:

```bash
cp /home/robot/slam_ws/build/lio_sam_hesai/lio_sam_hesai_mapOptimization \
   /home/robot/shutdown_auto_save_evidence/new/lio_sam_hesai_mapOptimization
ls -l /home/robot/shutdown_auto_save_evidence/old/lio_sam_hesai_mapOptimization \
      /home/robot/shutdown_auto_save_evidence/new/lio_sam_hesai_mapOptimization
cmp /home/robot/shutdown_auto_save_evidence/old/lio_sam_hesai_mapOptimization \
    /home/robot/shutdown_auto_save_evidence/new/lio_sam_hesai_mapOptimization
echo "cmp_exit=$?"
```

Expected: 两个文件都存在且大小非 0；`cmp_exit` 非 0 且 `cmp` 打印首个差异字节位置（例如 `... differ: byte 12345, line 1`），证明新二进制确实是重编产物，而不是旧文件被复制了两次。

- [ ] **Step 2: 新二进制正向 smoke（SIGINT 后零写入）**

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_smoke
cp /home/robot/slam_ws/src/lio_sam_hesai/config/mapping.yaml \
   /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml
sed -i 's#^      directory: .*#      directory: "/shutdown_auto_save_smoke/"#' \
  /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml
grep -n '^      directory:' /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml
rg -n 'enabled:' /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml
echo "fixture_has_no_enabled_key=$?"
export HOME=/home/robot
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 run lio_sam_hesai lio_sam_hesai_mapOptimization --ros-args \
  --params-file /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml \
  > /home/robot/shutdown_auto_save_evidence/run/new-positive.log 2>&1 &
wrapper_pid=$!
echo "$wrapper_pid" > /home/robot/shutdown_auto_save_evidence/run/new-positive.pid
sleep 12
node_pid=$(pgrep -P "$wrapper_pid" -f lio_sam_hesai_mapOptimization | head -n 1)
echo "node_pid=$node_pid"
test -n "$node_pid"
kill -INT "$node_pid"
for _ in $(seq 1 40); do
  kill -0 "$wrapper_pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$wrapper_pid" 2>/dev/null; then
  echo "wrapper_still_running=1"
  kill -KILL "$node_pid" 2>/dev/null || true
  kill -KILL "$wrapper_pid" 2>/dev/null || true
else
  echo "wrapper_still_running=0"
fi
wait "$wrapper_pid"
echo "new_exit=$?"
test ! -e /home/robot/shutdown_auto_save_smoke
echo "smoke_dir_absent=$?"
grep -c "Saving map to pcd files" \
  /home/robot/shutdown_auto_save_evidence/run/new-positive.log || true
touch /home/robot/shutdown_auto_save_evidence/run/new-positive.marker
```

**为什么必须用 `final-mapping.yaml` 而不是仓库原始 `config/mapping.yaml`：** 原始 YAML 的导出目录是 `/home/robot/Downloads/LOAM_hesai/`，与被观察的 `/home/robot/shutdown_auto_save_smoke` 无关，那样 `smoke_dir_absent` 会恒为真——即使关闭保存代码仍在也会“通过”。`final-mapping.yaml` 是**改动后**的仓库契约加一个可观察导出目录，所以“目录始终不出现”才真正等价于“退出时零写入”。fixture 还必须确认已不含 `enabled:`（`fixture_has_no_enabled_key=1`），否则跑的就是 fail-closed 分支而不是正常退出分支。

Expected: `grep` 输出一行 `      directory: "/shutdown_auto_save_smoke/"`；`rg -n 'enabled:'` 无输出且 `fixture_has_no_enabled_key=1`；`node_pid=` 非空；`wrapper_still_running=0`；`new_exit=0`；`smoke_dir_absent=0`（目录不存在，因为新代码不再 `mkdir`）；`grep -c` 输出 `0`（日志中没有 `Saving map to pcd files`）。

- [ ] **Step 3: 新二进制 fail-closed smoke（旧参数必须拒绝启动）**

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_smoke
export HOME=/home/robot
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 run lio_sam_hesai lio_sam_hesai_mapOptimization --ros-args \
  --params-file /home/robot/shutdown_auto_save_evidence/run/legacy-enabled.yaml \
  > /home/robot/shutdown_auto_save_evidence/run/new-fail-closed.log 2>&1 &
wrapper_pid=$!
echo "$wrapper_pid" > /home/robot/shutdown_auto_save_evidence/run/new-fail-closed.pid
sleep 8
if kill -0 "$wrapper_pid" 2>/dev/null; then
  echo "still_running=1"
  node_pid=$(pgrep -P "$wrapper_pid" -f lio_sam_hesai_mapOptimization | head -n 1)
  if [ -n "$node_pid" ]; then
    kill -INT "$node_pid"
  else
    kill -INT "$wrapper_pid"
  fi
  for _ in $(seq 1 20); do
    kill -0 "$wrapper_pid" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$wrapper_pid" 2>/dev/null; then
    kill -KILL "$wrapper_pid" 2>/dev/null || true
  fi
else
  echo "still_running=0"
fi
wait "$wrapper_pid"
echo "fail_closed_exit=$?"
grep -n "was removed with Ctrl+C automatic map saving" \
  /home/robot/shutdown_auto_save_evidence/run/new-fail-closed.log
grep -n "/lio_sam/save_map" \
  /home/robot/shutdown_auto_save_evidence/run/new-fail-closed.log
test ! -e /home/robot/shutdown_auto_save_smoke
echo "smoke_dir_absent=$?"
touch /home/robot/shutdown_auto_save_evidence/run/new-fail-closed.marker
```

Expected: `still_running=0`（8 秒内进程已自行退出，说明 fail-closed 真的在启动期生效）；`fail_closed_exit` 为非 0；两条 `grep -n` 都能命中 `map_save.enabled was removed with Ctrl+C automatic map saving; use /lio_sam/save_map instead`；`smoke_dir_absent=0`。若 `still_running=1`，说明 guard 没有在启动期触发，必须回到 Task B 定位。

失败形态说明：`ParamServer` 在成员初始化列表里调用 `LoadSlamConfig(*this, SlamMode::kMapping)` 且没有 try/catch（`include/lio_sam/utility.hpp:105-110`），所以异常会从节点构造函数抛出、无人捕获而 abort，libstdc++ 默认 terminate handler 会把 `what()` 打到 stderr，因此 `grep` 能命中迁移提示。典型观测是 `ros2 run` 打印 `[ros2run]: Aborted` 且 `fail_closed_exit=134`，但这只是观测值：**判据是“退出码非 0 + 迁移提示 + 零写入”，不得把 134 或任何具体非零码写成契约。**

- [ ] **Step 4: `/lio_sam/save_map` 服务接口 smoke（注册与类型）**

本步只验证服务被注册、类型正确，**不是真实导出回归**：不调用服务、不提供 bag，因此不能声称“地图导出仍然工作”。真实 bag 导出未跑，Task E 的 10.3 必须如实记录。

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_smoke
export HOME=/home/robot
source /opt/ros/humble/setup.bash
source /home/robot/slam_ws/install/setup.bash
ros2 run lio_sam_hesai lio_sam_hesai_mapOptimization --ros-args \
  --params-file /home/robot/shutdown_auto_save_evidence/run/final-mapping.yaml \
  > /home/robot/shutdown_auto_save_evidence/run/service.log 2>&1 &
wrapper_pid=$!
echo "$wrapper_pid" > /home/robot/shutdown_auto_save_evidence/run/service.pid
sleep 20
ros2 service list > /home/robot/shutdown_auto_save_evidence/run/service-list.txt
grep -n "lio_sam/save_map" /home/robot/shutdown_auto_save_evidence/run/service-list.txt
ros2 service type /lio_sam/save_map
node_pid=$(pgrep -P "$wrapper_pid" -f lio_sam_hesai_mapOptimization | head -n 1)
echo "node_pid=$node_pid"
test -n "$node_pid"
kill -INT "$node_pid"
for _ in $(seq 1 40); do
  kill -0 "$wrapper_pid" 2>/dev/null || break
  sleep 1
done
if kill -0 "$wrapper_pid" 2>/dev/null; then
  echo "wrapper_still_running=1"
  kill -KILL "$node_pid" 2>/dev/null || true
  kill -KILL "$wrapper_pid" 2>/dev/null || true
else
  echo "wrapper_still_running=0"
fi
wait "$wrapper_pid"
echo "service_exit=$?"
touch /home/robot/shutdown_auto_save_evidence/run/service.marker
```

Expected: `grep` 命中 `lio_sam/save_map`；`ros2 service type` 输出 `lio_sam_hesai/srv/SaveMap`；`wrapper_still_running=0`；`service_exit=0`。这就是“服务接口 smoke”的全部判据——注册与类型，不含导出。若 DDS/发现层在该机器上不可用，记录 `ros2 service list` 的真实输出与具体原因，**不得**把未跑成写成通过。

- [ ] **Step 5: 确认 smoke 目录未被重建并汇总证据**

Run:

```bash
test ! -e /home/robot/shutdown_auto_save_smoke
echo "smoke_dir_absent=$?"
test -f /home/robot/shutdown_auto_save_evidence/old-export/cloudGlobal.pcd
ls -l /home/robot/shutdown_auto_save_evidence/run/
```

Expected: `smoke_dir_absent=0`；旧输出归档仍含 `cloudGlobal.pcd`；`run/` 下能看到
`old-positive.log/marker`、`new-positive.log/marker`、`new-fail-closed.log/marker`、
`service.log/marker`、对应的 `*.pid`、`baseline-ctest.txt`、`baseline-tokens.txt`、
`guard-literals.txt`、`mapping.yaml.bak`、`mapping.yaml.injected`，以及两个参数 fixture
`legacy-enabled.yaml` 与 `final-mapping.yaml`（`final-ctest.txt` 在 Task E 才产生）。
本任务不读取、删除或断言 `/home/robot/Downloads/LOAM_hesai` 等真实导出目录的状态。

### Task E: 最终门禁与 PROJECT_STATUS 更新

**Files:**
- Modify: `docs/PROJECT_STATUS.md`（5 处：五、后续工作列表；8 节“遗留与注意事项”；9.7 范围外遗留；末尾“仍存的非本轮项”；文件末尾追加第 10 节）

**行号说明：** 本节引用的行号（103 / 248 / 332 / 421-422）都指**本轮编辑前**的 `docs/PROJECT_STATUS.md`（共 422 行）。每完成一步，文件行数会变化，后续步骤的行号会平移；**以引用的原文片段为准定位，行号仅作参考**。这些改动都不要改动手册第 1-9 节的其他结论。

- [ ] **Step 1: 跑完整约定门禁**

Run:

```bash
export HOME=/home/robot
source /opt/ros/humble/setup.bash
cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1
set -o pipefail
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  --output-on-failure -j1 \
  -E '^(copyright|cpplint|flake8|lint_cmake|uncrustify|xmllint)$' \
  | tee /home/robot/shutdown_auto_save_evidence/run/final-ctest.txt
```

Expected: `100% tests passed, 0 tests failed out of 44`，与 Phase 0 基线计数一致（测试只增不改名，数量应不变）。**以终端实际输出为准**：若实际不是 44/44，停止本任务并记录差异，先按第 0 节第 2 条修订本计划的 44 与 Step 6 模板后再继续；不得把未修订的数字写进 `docs/PROJECT_STATUS.md`。

- [ ] **Step 2: 更新“五、后续工作”列表**

在 `docs/PROJECT_STATUS.md` 中把第 103 行：

```markdown
2. **退出自动保存产物收敛**：Ctrl+C 路径仍写无人回读的 `cloud*.pcd`，且不写 `sc_db.bin`；需要统一为定位和分块生成器可消费的产物契约。
```

整行删除，并把其后的 3、4 两条重新编号为 2、3（列表保持连续）。**不要**删除 P2.3 条目。

- [ ] **Step 3: 更新第 8 节“遗留与注意事项”中的历史结论**

把第 248 行：

```markdown
- 退出自动保存仍写 `cloudCorner.pcd` / `cloudSurf.pcd` / `cloudGlobal.pcd` / `cloudGround.pcd`，而服务保存写 `CornerMap.pcd` / `SurfMap.pcd` / `GlobalMap.pcd` / `GroundMap.pcd`。P2.2 不重命名磁盘文件，因此四个历史名字已收敛到 `MapArtifactLayout` 的 `Saved*Cloud()` 访问器。**这看起来是真实缺陷**：没有任何代码回读 `cloud*.pcd`，只靠 Ctrl+C 保存的地图无法被定位或分块生成器加载（它们找 `GlobalMap.pcd`）。
```

替换为：

```markdown
- ~~退出自动保存仍写 `cloudCorner.pcd` / `cloudSurf.pcd` / `cloudGlobal.pcd` / `cloudGround.pcd`，而服务保存写 `CornerMap.pcd` / `SurfMap.pcd` / `GlobalMap.pcd` / `GroundMap.pcd`……只靠 Ctrl+C 保存的地图无法被定位或分块生成器加载（它们找 `GlobalMap.pcd`）。~~ **该缺陷已于 2026-09-15 按“删除而非重命名”解决**：Ctrl+C 路径、`map_save.enabled` / `savePCD` 参数和 `MapArtifactLayout::Saved*Cloud()` 全部删除，`/lio_sam/save_map` 成为唯一导出入口。当时选择不重命名磁盘文件，因此 `CornerMap.pcd` / `SurfMap.pcd` / `GlobalMap.pcd` / `GroundMap.pcd` 的名字保持不变。
```

- [ ] **Step 4: 更新 9.7“范围外遗留”条目**

把第 332 行：

```markdown
- **退出自动保存的产物无人消费**：`map_save.enabled` 只控制这一条路径，写出的 `cloudCorner/cloudSurf/cloudGlobal/cloudGround.pcd` 没有任何回读方，且不写 `sc_db.bin`；只靠 Ctrl+C 保存的地图无法被定位或分块生成器加载。修它需要重命名磁盘文件，超出 P2.2 范围。
```

替换为：

```markdown
- ~~**退出自动保存的产物无人消费**：`map_save.enabled` 只控制这一条路径，写出的 `cloudCorner/cloudSurf/cloudGlobal/cloudGround.pcd` 没有任何回读方，且不写 `sc_db.bin`。~~ **已于 2026-09-15 关闭**：不采用“重命名磁盘文件”的方案，而是直接删除该路径；`map_save.enabled` 与 `savePCD` 现在在启动期 fail-closed 拒绝并提示改用 `/lio_sam/save_map`。验收证据见第 10 节。
```

- [ ] **Step 5: 更新末尾“仍存的非本轮项”**

把第 421-422 行：

```markdown
**仍存的非本轮项**：P2.3，以及 9.7 的退出自动保存磁盘产物、`TightCouplingKF` 运行期
`LOG(FATAL)` 桩、四项 lint 基线——本轮未清零，也不宣称清零。
```

替换为：

```markdown
**仍存的非本轮项**：P2.3、`TightCouplingKF` 运行期 `LOG(FATAL)` 桩、四项 lint 基线——
本轮未清零，也不宣称清零。9.7 的退出自动保存磁盘产物已按第 10 节关闭，不再属于遗留项。
```

- [ ] **Step 6: 追加第 10 节验收记录（模板中所有数字都是确定性契约值）**

在文件末尾追加下面的内容。模板里的数字全部是本计划固定的契约值：约定门禁 44/44、旧行为 wrapper 判据通过、新行为退出码 0 且 banner 计数 0、fail-closed 非零退出。**模板不含任何待填占位符**；若 Task E Step 1 或 Task D 的实际输出与这些数字不符，先停止并按第 0 节第 2 条修订本计划，不得直接写入不同数字。

```markdown

## 十、退出自动保存删除（2026-09-15）

按 [计划](superpowers/plans/2026-09-15-remove-shutdown-auto-save.md) 执行，代码基线 `c9aa44a`（计划提交 `docs: plan shutdown auto-save removal` 只新增本计划文件，不含实现改动）。目标：Ctrl+C 不再写任何地图文件、不改动导出目录，`/lio_sam/save_map` 成为唯一导出入口，删除的参数在启动期 fail-closed 拒绝。

### 10.1 改动

- 删除 `src/mapOptimization.cpp` 的 `save_map_enabled_` 成员、构造赋值，以及 `visualizeGlobalMapThread()` 退出后的整段导出代码（`rm -r` / `mkdir` / 5 次 PCD 写入与 Z 偏移导出）。
- 删除仅供该路径使用的 `MapArtifactLayout::kSaved*CloudName` 常量与 `Saved*Cloud()` 访问器（`include/common/map_artifact_layout.h`、`src/common/map_artifact_layout.cpp`）。
- 删除 `MapSaveConfig::enabled`、`map_save.enabled` 读取（`savePCD` 别名随之失效）、`config/mapping.yaml` 的 `enabled` leaf，以及 `test_unified_parameter_schema.cpp` 的 `{"savePCD", "map_save.enabled", ...}` 契约行。
- 新增 `RejectRemovedMappingParameters()`：在 `LoadSlamConfig` 的任何 `Load*` 之前读取原始 override map，命中 `map_save.enabled` 或 `savePCD` 时抛出 `was removed with Ctrl+C automatic map saving; use /lio_sam/save_map instead`。
- 新增契约测试：`SlamConfig.RejectsRemovedShutdownAutoSaveParameters`、`UnifiedParameterSchema.ShutdownAutoSaveImplementationIsAbsent`、`UnifiedParameterSchema.ShutdownAutoSaveContract`（YAML/launch 反向契约 + guard 字面量精确计数）。
- 未新增脚本；生产路径零匹配与 guard 文件两处字面量各恰好 1 次由上述静态契约测试与 Task C 的一次性 `rg` allowlist 扫描加注入自检验证。

### 10.2 验证证据

- 基线构建：`cmake --build /home/robot/slam_ws/build/lio_sam_hesai -- -j1`，退出码 0。
- 约定 CTest 门禁（排除 `copyright`/`cpplint`/`flake8`/`lint_cmake`/`uncrustify`/`xmllint`）：基线 44/44 通过，改动后 44/44 通过；完整输出见 `run/baseline-ctest.txt` 与 `run/final-ctest.txt`。
- 旧二进制阳性对照：`/home/robot/shutdown_auto_save_evidence/old/` 的 `c9aa44a` 二进制配 `enabled: true`，SIGINT 后写入 `trajectory.pcd`、`transformations.pcd`、`cloudCorner.pcd`、`cloudSurf.pcd`、`cloudGlobal.pcd`，wrapper 判据通过（`wrapper_still_running=0`），退出码 0。
- 新二进制正向 smoke：同一固定导出目录在 SIGINT 后仍不存在，退出码 0，日志中 `Saving map to pcd files` 出现 0 次。
- 新二进制 fail-closed smoke：`enabled: true` 参数文件启动即退出，退出码非 0，日志含 `was removed with Ctrl+C automatic map saving; use /lio_sam/save_map instead`，导出目录不存在；具体非零码不作为契约。
- `/lio_sam/save_map` 服务接口 smoke：`ros2 service list` 含 `lio_sam/save_map`，`ros2 service type` 为 `lio_sam_hesai/srv/SaveMap`；只验证注册与类型，未调用导出。
- 静态 allowlist 扫描（Task C 的一次性 `rg`，非脚本）：生产路径 `include/ src/ config/ launch/` 的历史 token 零匹配，guard 文件 `map_save.enabled` 与 `savePCD` 各恰好 1 次；注入自检先命中、恢复后无匹配，`git status --short` 干净。
- 证据文件位置：`/home/robot/shutdown_auto_save_evidence/`（`old/`、`new/`、`run/`）。

### 10.3 未做

- 未 push。
- 未做全量 lint 治理，四项既有 lint 基线（`flake8`、`lint_cmake`、`uncrustify`、`xmllint`）未清零。
- 未做真实 bag 导出：`/lio_sam/save_map` 只做了服务接口 smoke（注册与类型），没有在真实 bag 回放中调用服务导出地图。
```

- [ ] **Step 7: 校验文档改动**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
echo "--- 后续工作列表仍是连续编号（应为 1./2./3. 三条）---"
sed -n '/^## 五、后续工作/,/^## 六、/p' docs/PROJECT_STATUS.md | grep -n '^[0-9]\.'
echo "--- 不应再有“退出自动保存产物收敛”这一待办项 ---"
rg -n '退出自动保存产物收敛' docs/PROJECT_STATUS.md
echo "leftover_open_item_exit=$?"
echo "--- 保留项仍在 ---"
rg -n 'P2\.3 地图产物版本化与兼容校验（暂缓）|TightCouplingKF` 产品决策|历史 lint 基线治理' docs/PROJECT_STATUS.md
echo "--- 第 10 节存在且为确定性数字 ---"
rg -n '^## 十、退出自动保存删除|基线 44/44 通过|改动后 44/44 通过' docs/PROJECT_STATUS.md
echo "--- 服务接口 smoke 与未跑项如实记录 ---"
rg -n '服务接口 smoke|未做真实 bag 导出' docs/PROJECT_STATUS.md
echo "--- 计划链接有效且计划文件受跟踪 ---"
rg -n --fixed-strings 'superpowers/plans/2026-09-15-remove-shutdown-auto-save.md' docs/PROJECT_STATUS.md
git ls-files --error-unmatch docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
echo "plan_tracked_exit=$?"
```

Expected: 第一段列出 `1.`、`2.`、`3.` 三条连续编号（P2.3、`TightCouplingKF`、lint 基线）；`leftover_open_item_exit=1`（该待办项已删除）；保留项三条各命中一次；`## 十、退出自动保存删除（2026-09-15）`、`基线 44/44 通过`、`改动后 44/44 通过` 各命中；`服务接口 smoke` 与 `未做真实 bag 导出` 各命中；计划链接命中一次且 `plan_tracked_exit=0`。

- [ ] **Step 8: 暂存并提交文档**

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git add -u docs/PROJECT_STATUS.md
git status --short
git diff --cached --stat
git commit -m "docs: record shutdown auto-save removal"
```

Expected: 只暂存并提交 `docs/PROJECT_STATUS.md`（`M`）。

### Task F: 最终自检

**Files:**
- 只读

- [ ] **Step 1: 工作区与提交历史**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git status --short
git log -5 --oneline
git diff --check
```

Expected: `git status --short` 无输出；`git log -5 --oneline` 由上到下恰好是

```text
docs: record shutdown auto-save removal
fix(config): reject removed shutdown auto-save parameters
refactor(mapping): delete shutdown auto-save runtime path
docs: plan shutdown auto-save removal
c9aa44a docs: design shutdown auto-save removal
```

即本轮共 4 个新提交（计划提交、Task A、Task B、Task E 文档）；`git diff --check` 退出码 0。若中途按第 0 节第 2 条提交过 `docs: revise shutdown auto-save removal plan`，它会位于计划提交与 Task A 之间，属预期修订，此时提交总数为 5。

- [ ] **Step 2: 复查只动了允许的文件**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
git diff --name-only c9aa44a..HEAD | sort
```

Expected: 恰好 10 行（9 个实现/文档文件 + 本计划文件）：

```text
config/mapping.yaml
docs/PROJECT_STATUS.md
docs/superpowers/plans/2026-09-15-remove-shutdown-auto-save.md
include/common/map_artifact_layout.h
include/common/slam_config.h
src/common/map_artifact_layout.cpp
src/common/slam_config.cpp
src/mapOptimization.cpp
test/test_slam_config.cpp
test/test_unified_parameter_schema.cpp
```

本计划文件出现在这里是因为 Task 0.2 已把它提交为受跟踪工件，`c9aa44a..HEAD` 天然包含它；若改用计划提交之后的范围比较，则恰好是上面除计划文件外的 9 行。`scripts/` 下不得出现任何路径（本轮不新增脚本），出现第 11 个路径即越界修改，必须回退该文件后再提交。

- [ ] **Step 3: 最终扫描与测试收口**

Run:

```bash
cd /home/robot/slam_ws/src/lio_sam_hesai
rg -n --no-heading \
  'save_map_enabled_|Saved(Corner|Surface|Global|Ground)Cloud|kSaved(Corner|Surface|Global|Ground)CloudName|cloud(Corner|Surf|Global|Ground)\.pcd' \
  include src config launch
echo "final_scan_exit=$?"
rg -o --no-filename 'map_save\.enabled|"savePCD"' src/common/slam_config.cpp | sort | uniq -c
ctest --test-dir /home/robot/slam_ws/build/lio_sam_hesai \
  -R '^(test_slam_config|test_unified_parameter_schema|test_map_artifact_layout)$' \
  --output-on-failure
```

Expected: `final_scan_exit=1`（生产路径零匹配）；`uniq -c` 恰好两行、计数各为 1；3/3 测试通过。到这里为止的每一条通过声明都必须有本轮实际终端输出作为依据；未执行的项（例如 DDS 不可用时的服务接口 smoke、未跑的真实 bag 导出）一律按“未运行 + 具体原因”记录，不得写成通过。

---

## 依赖顺序与提交边界

| 顺序 | 任务 | 提交信息 | 依赖 |
| --- | --- | --- | --- |
| 1 | Task 0.2（Phase 0 内） | `docs: plan shutdown auto-save removal` | 基线 `c9aa44a` |
| 2 | Phase 0 其余步骤 | 无提交 | Task 0.2（计划已受跟踪，后续 diff 才包含它） |
| 3 | Task A | `refactor(mapping): delete shutdown auto-save runtime path` | Phase 0 |
| 4 | Task B | `fix(config): reject removed shutdown auto-save parameters` | Task A（`Saved*` 与运行期路径已删，配置删除才自洽） |
| 5 | Task C | 无提交 | Task B（一次性扫描要求生产路径零匹配） |
| 6 | Task D | 无提交 | Task B + Task C（需要新二进制与 guard 已生效） |
| 7 | Task E | `docs: record shutdown auto-save removal` | Task D（证据来自 D 与 Phase 0） |
| 8 | Task F | 无提交 | Task E |

**本轮共 4 个提交**：Task 0.2 的计划提交、Task A、Task B、Task E 的文档提交；Task C/D/F 只读或只写证据目录，不产生提交。`docs/superpowers/plans/` 下只有 Task 0.2 一次强制暂存（`git add -f`），不会再有第二次。

**Task A 与 Task B 的顺序不可交换：** Task B 删除 `MapSaveConfig::enabled` 之前，Task A 必须先删掉它的唯一运行期读者；否则 Task A 单独编译时 `save_map_enabled_` 仍在读一个已删字段。反向顺序（先删配置再删运行期）会让 Task A 无法独立编译，正是本计划要避免的。
