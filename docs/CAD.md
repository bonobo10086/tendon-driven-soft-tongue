# CAD 源码和版本

模型以 Python 参数和三角网格布尔运算构建，依赖 NumPy、trimesh、manifold3d 和 Pillow。源码不是 Fusion 原生参数化工程，也不提供 STEP 特征树。

## 优先使用已导出的图纸

[当前打印清单](../hardware_prototype/PRINT_INDEX.md)是唯一的打印入口。`stl/`、`revision_v3/` 和 `revision_v4/` 同时保留旧模、装配参考和计算缓存，是因为后续版本读取它们作为基线，不是要求新用户把全部 STL 打一遍。

| 版本 | 设计作用 |
|---|---|
| V2 | 根据 SOFA 舌体表面缩放建立模腔、根嵌件和尖锚板 |
| V3 | 调整 L 支架和路线参考 |
| V4 | 引入开放装配槽、可拆根/尖端盖 |
| V5 | 保留舌体和配合结构，改为倒置浇注，移动灌注和排气通道 |

整根弯芯棒卡滞是实际遇到的制造问题，不能只用数字几何核查来否定。后续改为短直针端部定位，软管中段不再插整根弯棒。

## 可选的源码核查

建议 Python 3.12，在单独虚拟环境中安装：

```text
python -m pip install -r requirements-cad.txt
python hardware_prototype/cad/check_revision_v5_independent.py
```

检查脚本会重新写入 V5 的核查报告，但不连接硬件。几何检查不检查硅胶固化、摩擦或舵机输出力。

确实需要重建 V5 模具时，在**仓库副本**运行：

```text
python hardware_prototype/cad/build_revision_v5.py
```

该脚本依赖保留的 V2/V4 基线，按原路径生成输出，并会更新几何报告。别直接运行旧 `build_printables.py` 的主入口来覆盖基线；库导入与直接执行主入口不是一回事。

Pillow 在这些模块中用于历史制图工具。现有部分制图函数默认 Windows 字体；在其他系统重画中文字图时需要自行替换字体路径。纯 V5 几何核查不需要打开绘图窗口。

`requirements-cad.txt` 是版本范围，不是已穷尽验证的跨平台锁文件；本次发布的实际核查结果见 [VALIDATION.md](VALIDATION.md)。
