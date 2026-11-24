# 来源、贡献与许可

## SOFA 模型资源

`assets/mesh/` 和 `assets/textures/` 的 6 个文件来自 SOFA 的 Dentistry_Haptics 教程，发布整理时逐字节核对了本机 SOFA 25.12.00 安装内的同名原文件。纹理在原项目路径是 `data/pics/`，在本仓库归入 `assets/textures/`；几何网格和纹理内容未修改。

来源仓库：[sofa-framework/sofa](https://github.com/sofa-framework/sofa)。本地发行包记录的源码提交为 `4d6c2b8e8cd976836c5e071e9db14f1963d84cc9`。逐文件来源和 SHA-256 见 [UPSTREAM.json](../assets/UPSTREAM.json)。

上游 README 声明 SOFA 通常采用 LGPL-2.1-or-later，特定应用目录或另有单独许可证的目录除外。本仓库随附上游 [LICENSE-LGPL.md](../assets/LICENSE-LGPL.md) 与 [作者名单](../assets/SOFA-AUTHORS.txt)，不把这些资源重新许可为 MIT，也不声称人体网格由本项目原创。CAD 模腔直接派生自上述舌体几何，复用时同样需要考虑其上游许可。

## 文献参考

Nobutsuna Endo, 2021. *Deformation model and experimental evaluation of a contractable and bendable wire-pulling mechanism with embedded soft tubes for a robotic tongue*. ROBOMECH Journal 8, 6. DOI: [10.1186/s40648-021-00193-6](https://doi.org/10.1186/s40648-021-00193-6)。

参考的是内嵌软管中的拉索驱动原理和制造思路，不是复刻作者提供的 CAD。该论文的两导管试件、材料配方与实验数据不能标为本项目四导管舌体的实测结果。仓库没有复制论文图片或 PDF。

## 本项目的工作

项目将 SOFA 教程网格用于交互式 FEM 舌体，加入固定区域、连续目标权重和可视化诊断；设计多轮模具、端盖和安装件；实施 3D 打印、试装、硅胶浇注与 Arduino 调试。

开发过程中使用 ChatGPT / Codex 辅助编程、网格建模、核查与文档整理。项目作者承担设计选择、实物制造与调试；不将 AI 辅助工作表述成论文团队的官方复现成果。

## 本仓库的发布许可状态

本次整理保留第三方文件的既有许可，没有未经作者选择就为全部原创代码、文档或图纸新增统一的开源许可证。公开可读不等于自动授权任意复制与商业使用。若项目作者随后选择开源许可，应先确认与上游及派生几何许可相容，再增加根目录 LICENSE；本文件不能替代上游许可证。
