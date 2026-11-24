# 仿真运行与控制

## 环境

项目在 Windows 的 SOFA 25.12 中运行过；SofaPython3 插件与 Python 的版本必须匹配。普通 `pip install sofa` 不能替代完整 SOFA 运行环境。本次整理不升级原有 SOFA，也不重新安装驱动。

原场景使用 `ParallelCGLinearSolver` 和 `ParallelTetrahedronFEMForceField`，因此要求 SOFA 的 MultiThreading 插件。Qt 设置面板由 Sofa.Qt 提供。

## 启动

用已经安装的 Python 调用启动脚本：

```powershell
python tools/launch_sofa.py --sofa-dir "D:\SOFA\SOFA_v25.12.00_Win64" --python-dir "D:\SOFA\Python312"
```

启动脚本本身不导入 Sofa，而是调用匹配安装里的 `runSofa`。也可以在 SOFA GUI 中加载 SofaPython3 后，直接打开仓库根目录的 `tongue_motion.py`。

如果想用双击入口，在命令提示符中设定自己的安装目录后启动：

```bat
set "SOFA_ROOT=D:\SOFA\SOFA_v25.12.00_Win64"
set "SOFA_PYTHON_ROOT=D:\SOFA\Python312"
START_TONGUE.cmd
```

这些变量只在当前窗口内有效；可以自行设置用户环境变量，但不需要改场景源码。Linux 可用同一 Python 启动脚本指定 SOFA 路径，不过本项目没有在 Linux 上完成运行验收。

## 操作

1. 场景载入后默认自动循环；若暂停，点击 Animate。
2. 点击三维画面取得焦点。在 Qt GUI 中，若字母键被界面截获，使用 Ctrl+字母。
3. W/S 上下、A/D 左右、E/Q 前伸后缩，X 或 0 回中。
4. `TongueControl` 的 `Tongue movement` 面板也能切换 `automaticMotion`，并直接修改三个手动目标。

`TARGET_STEP=0.08`、`TARGET_LIMIT=0.55` 是模型坐标下的命令值，不能读成真实舌尖位移的毫米数。`visibleSurfaceDisplacement` 是渲染表面的最大位移，不是实体传感器数据。

## 模型怎么工作

OBJ 表面经 MeshTetraStuffing 生成体网格；舌根 ROI 固定，舌尖向舌中使用渐变权重。控制器移动隐藏目标，RestShapeSpringsForceField 驱动 FEM 舌体跟随，再经 BarycentricMapping 映射到可见表面。

这避免了直接改部分可视顶点造成的不连续外观，但不是舌肌或拉索张力模型。头部与牙齿仅用于显示，没有舌体碰撞处理；材料与实体硅胶尚未标定。
