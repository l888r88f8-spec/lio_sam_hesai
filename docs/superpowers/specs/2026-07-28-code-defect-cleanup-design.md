# 子项目 F：代码缺陷清理设计

**日期：** 2026-07-28  
**状态：** 已确认  
**范围：** `lio_sam_hesai` 已知低风险代码缺陷

## 1. 目标

在不改变对外 ROS 接口、参数、话题或正常算法行为的前提下，清理
`docs/HANDOFF.md` 指定的三个已知缺陷：

1. 修复 `IVoxMap::AddPoints(world_points, nearest_points)` 遇到单个无需降采样点时
   提前终止整批处理的问题。
2. 删除未加入任何 CMake 构建目标的遗留源文件
   `src/slam/frontend.cpp` 和 `src/slam/loop_closure.cpp`。
3. 移除公共头文件 `include/lio_sam/utility.hpp` 中的
   `using namespace std;`，并为该头文件内依赖的标准库类型补齐 `std::` 限定。

## 2. 非目标

- 不清理其他文件中的命名空间、格式或 lint 问题。
- 不重构 iVox 数据结构、邻域搜索或降采样策略。
- 不恢复、迁移或替代两份死代码中的功能。
- 不修改建图或重定位的 ROS 接口、配置参数和运行时线程模型。

## 3. 设计

### 3.1 iVox 批量添加修复

保留现有两阶段收集逻辑：

- 需要降采样的点进入 `points_to_add`。
- 无需降采样的点进入 `point_no_need_downsample`。
- 循环结束后，两个集合分别交给单参数 `AddPoints` 写入 iVox。

当某个点满足“无需降采样”条件时，只结束当前迭代，继续处理批次中的后续点。
实现上把当前的 `return` 改为 `continue`。不调整判断条件、阈值或容器结构。

### 3.2 死代码删除

直接删除 `src/slam/frontend.cpp` 和 `src/slam/loop_closure.cpp`。两者当前不属于
任何 CMake 目标，也没有其他源文件包含或链接其实现，因此无需替代实现或构建规则变更。

### 3.3 公共头文件命名空间隔离

删除 `include/lio_sam/utility.hpp` 的全局 `using namespace std;`。仅在该头文件中把
现有未限定的标准库类型改为显式 `std::` 名称，例如 `std::string` 和
`std::vector<double>`。不引入新的 namespace alias，也不顺带改动包含该头文件的源文件。

## 4. 测试策略

所有生产改动遵循 RED → GREEN：

1. 新增 iVox 行为回归测试。测试构造至少两个输入点：第一个点触发“无需降采样”
   分支，第二个点需要正常加入。修复前测试应证明第二个点不可查询；修复后应能通过
   iVox 最近邻查询找到第二个点。
2. 新增代码卫生契约测试。测试读取仓库源文件，断言公共头文件不含
   `using namespace std;`，并断言两份死代码不存在。修复前应因三项契约未满足而失败。
3. 将新测试注册到现有 `ament_cmake_gtest` 测试体系。
4. 修复后运行新测试、全部非历史-lint测试和包级构建。

项目已有的 CMake/PCL/Boost/TBB 警告和已知 lint 债务不属于本子项目的失败判据；
新增或受影响目标不得产生新的编译错误或测试失败。

## 5. 验收标准

- iVox 回归测试在旧实现上失败，在 `continue` 修复后通过。
- 代码卫生测试在清理前失败，在删除死代码并移除头文件命名空间污染后通过。
- 两份死代码从仓库中删除，且 CMake 不引用它们。
- `utility.hpp` 不再包含 `using namespace std;`，其中标准库类型均显式限定。
- `lio_sam_hesai` 包构建成功。
- 全部非历史-lint测试通过，现有测试无回归。

## 6. 风险与回退

改动集中在一个未使用的 iVox 重载、两个未编译文件和一个公共头文件的名称限定，
运行时风险低。主要风险是遗漏 `utility.hpp` 内依赖命名空间注入的标准库名称，
由完整包构建覆盖。若出现无法解释的行为回归，可按独立提交逐项回退对应改动。
