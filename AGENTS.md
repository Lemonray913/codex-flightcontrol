# 项目速览

本项目当前工作区根目录为 `D:\Work\codex\codex-flightcontrol`，实际源码位于 `GHV_open-main/`。项目名为 **GHV_open**，是一个面向 Generic Hypersonic Vehicle（GHV，高超声速飞行器）的 MATLAB/Simulink 开源动力学与 GNC 仿真平台。代码和模型主要围绕 6 自由度动力学、气动建模、组合推进、地球/大气环境、轨迹设计以及控制算法示例展开。

许可证为 GPL-2.0，见 `GHV_open-main/LICENSE`。README 标注推荐 MATLAB R2023b+。

## 目录结构

- `GHV_open-main/init.m`：初始化入口。清空工作区，递归 `addpath` 当前工程目录，并调用 `GHV_Configuration()`。
- `GHV_open-main/GHV_Configuration.m`：飞行器基础参数、初始飞行状态和 Simulink Bus 初始化。
- `GHV_open-main/GHV_model/`：动力学、气动、推进、质心/惯量、地球环境等核心模型函数。
- `GHV_open-main/GHV_control/`：反馈线性化、滑模控制、自适应模糊/滑模控制等控制器代码和子模型。
- `GHV_open-main/GHV_trajectory/`：三自由度轨迹方程、参考攻角、参考飞行时序和轨迹设计 Simulink 模型。
- `GHV_open-main/GHV_analysis/`：升力、阻力、升阻比和最优攻角分析脚本。
- `GHV_open-main/Control_Schemes/`：不同控制方案的 Simulink 示例模型。
- `GHV_open-main/docs/`：论文、模型说明文档和仿真平台截图。
- `GHV_open-main/data/`、`cache/`：数据和缓存占位目录。

## 主要 Simulink 模型

- `GHV_open_equation.slx`：基于 MATLAB 公式实现的动力学仿真平台。
- `GHV_open_toolbox.slx`：基于 Simulink / Aerospace Toolbox 的动力学仿真平台。
- `GHV_open_VarialbeMass_toolbox.slx`：考虑发动机推力和变质量动力学的平台。
- `GHV_open_VarialbeMass_elliposid_toolbox.slx`：进一步考虑椭球地球的变质量平台。
- `Control_Schemes/*.slx`：包含自适应模糊、滑模、MIMO 自适应滑模等控制方案示例。
- `GHV_trajectory/*.slx`：姿态角、气流角和球面轨迹设计方程模型。

注意：文件名中 `VarialbeMass`、`elliposid` 等拼写沿用原项目，不要在无明确需求时重命名，否则可能破坏模型引用。

## 初始化与运行

常规使用方式是在 MATLAB 中进入 `GHV_open-main/`，执行：

```matlab
init
```

`init.m` 会递归加载工程路径并执行 `GHV_Configuration.m`。配置脚本会创建 `GHV_cfg` 结构体，并调用 `Simulink.Bus.createObject(GHV_cfg)` 生成供 Simulink 使用的总线对象。

## 核心配置

`GHV_Configuration.m` 中的重要默认值：

- 参考弦长 `c_ref = 24.38 m`
- 参考翼展 `b_ref = 18.29 m`
- 参考面积 `s_ref = 334.73 m^2`
- 质心到力矩中心距离 `x_cg = 2.9 m`
- 发动机到参考力矩中心距离 `x_cT = 23.16 m`
- 满载质量 `mass_full = 136077 kg`
- 初始马赫数 `Ma = 5`
- 初始高度 `altitude = 20000 m`
- 初始航迹倾角和攻角默认为 0
- 初始 LLA 为 `[19.6144722; 110.9510972; altitude]`
- 目标 LLA 为 `[38.87099; -77.05596; 0]`

## 核心模型说明

### 动力学与运动学

- `tra_dyn.m`：平动动力学，输入气动力、重力、速度、气流角、姿态角和角速度，输出速度导数和气流角导数。当前版本内部固定质量为 `136077 kg`，推力 `T = 0`。
- `tra_dyn_VarialbeMass.m`：变质量版本的平动动力学，显式输入推力和质量。
- `rot_dyn.m`：转动动力学，输入合外力矩和机体系角速度，输出角加速度。当前使用硬编码最大质量惯量。
- `rot_kin.m`：姿态运动学，计算欧拉角导数。
- `tra_kin.m`：平动运动学，计算位置/高度导数。
- `cg_inertia_variation.m`：根据当前质量计算质心位置、惯量矩阵以及惯量对质量的导数。

### 气动模型

- `Get_Aerodynamic.m`、`Aerodynamic_coefficients.m`：主气动模型，输入舵面偏角、气流角、速度、角速度、声速和密度，输出马赫数、气动力、气动力矩、动压和气动系数。
- 气动系数按马赫数分段：
  - `M < 1.15` 使用低速模型
  - `1.15 <= M < 1.35` 在低速/中速模型间线性过渡
  - `1.35 <= M < 3.9` 使用中速模型
  - `3.9 <= M < 4.1` 在中速/高速模型间线性过渡
  - `M >= 4.1` 使用高速模型
- 气动代码内部混用 SI 与英制单位以匹配 NASA/AIAA 公开数据，修改时务必留意 `ft`、`lb`、`N`、`N*m` 的换算。
- `Get_Aerodynamic_xcg.m` 支持外部输入质心位置；`Get_Aerodynamic_Simplified.m`、`Get_Aerodynamic_ident.m`、`Aerodynamic_coefficients_affine.m` 是简化、辨识或仿射形式的变体。
- `aero_forces_D_L.m` 与 `aero_forces_moments.m` 用气动系数和动压计算升阻力/力矩。

### 推进与环境

- `Propulsion_model.m`：组合发动机模型。输入油门 `PLA`、高度、马赫数、矢量推力偏角、配置和质心，输出推力、推力矩、比冲和质量变化率。
- 推进分段逻辑：
  - `Ma <= 2`：涡轮发动机段
  - `2 < Ma <= 6`：冲压/超燃冲压段
  - `Ma > 6`：火箭段
- `PLA` 约束在 `[0, 1]`，`Ma` 约束在 `[0, 24]`。
- `EarthEnvironment.m` 使用 `atmoscoesa` 获取 USSA76 标准大气参数，并按高度近似计算重力加速度。
- `geometric2geopotential.m` 做几何高度和位势高度转换。

## 控制与轨迹

- `Sliding_mode.m`：简单俯仰通道滑模控制，按攻角误差和攻角变化率生成左右升降舵偏角。
- `FBL_M.m`、`lie_solving.m`：反馈线性化相关实现和 Lie 导数求解。
- `adaptive fuzzy control/attitude_adaptive_smc.m`：MIMO 自适应滑模姿态控制器，输出左右舵和方向舵偏角，以及自适应权重变化率和滑模面。
- `adaptive fuzzy control/fuzzy_adaptive_controller.m`、`robust_control.m`、`math/L2Norm.m`：模糊自适应、鲁棒项和辅助数学函数。
- `Reference_Flight_Timeline.m`：参考飞行时序。默认前 `60 s` 维持 `alpha = 6 deg`、`PLA = 1`；随后用 `2 s` 过渡到基于马赫数的最佳升阻比攻角多项式，并关闭油门。
- `reference_alpha.m`、`three_dof_trajectory.m`、`three_dof_sphere_trajectory.m`：参考攻角与三自由度轨迹方程。

## 分析脚本

- `GHV_analysis/L_analysis.m`：升力分析。
- `GHV_analysis/D_analysis.m`：阻力分析。
- `GHV_analysis/L_D_analysis.m`：升阻比分析。
- `GHV_analysis/alpha_opt_LD.m`：给定马赫数范围内搜索最大升阻比对应的攻角。
- `GHV_model/model_analysis/propulsion_characteristics_analysis.m`：推进特性分析，配套 `Isp.xlsx` 和 `Isp.sfit`。

## 依赖与工具箱

从代码和模型名看，项目至少依赖：

- MATLAB R2023b 或更新版本
- Simulink
- Aerospace Toolbox（README 和 `GHV_open_toolbox.slx` 明确提到）
- `atmoscoesa` 函数可用的环境
- 使用控制方案模型时可能需要相关控制、模糊或 Simulink 附加工具箱，需以打开模型后的 MATLAB 报错为准

当前工作环境未验证 MATLAB 是否可用，也未运行仿真。

## 编码与文本注意事项

源码注释大多是中文，但在当前 PowerShell 输出中有明显乱码，推测存在 UTF-8/GBK 显示不匹配。编辑 `.m` 或 `.md` 文件时要小心保持原文件编码，避免大面积重写中文注释。

很多文件头部写有类似 “Read only, do not modify place!!!” 的提示，这是原作者注释风格。若任务只要求项目梳理，应避免改动核心模型源码。

## 修改建议

- 修改模型逻辑前，优先确认对应 Simulink 模型是否引用了函数名、变量名或总线对象。
- 不要随意移动或重命名 `.slx`、核心 `.m` 文件和目录，Simulink 引用可能依赖当前路径。
- 涉及气动、推进、质量/惯量的变更要同步检查单位制，尤其是英制和 SI 单位混用的位置。
- 改 `GHV_Configuration.m` 后应重新运行 `init`，确保工作区变量和 Simulink Bus 更新。
- `.gitignore` 已排除 `draft.m`、`*.mldatx`、`*.slxc`、`*.mat`、`slprj/`，不要把仿真缓存或大型临时数据纳入版本控制。

## 已知风险与 TODO 线索

- README 和部分注释在当前终端里显示乱码，阅读时可能需要在 MATLAB/编辑器中切换编码。
- `Propulsion_model.m` 中原作者标注 TODO：某些马赫数和油门条件下推力可能为负，怀疑是文献公式或实现问题。
- `tra_dyn.m`、`rot_dyn.m` 中存在硬编码质量/惯量；如果做变质量或参数化仿真，应优先使用变质量模型或配置结构体。
- 气动模型注释提到部分马赫数分段点通过线性过渡保持连续，且大攻角/高马赫下参数可靠性有限。
- 当前未发现自动化测试；验证通常需要 MATLAB/Simulink 打开模型并运行仿真。

## 代码提交与远程仓库协作

- 当前项目使用 Git 做版本管理，本地仓库根目录是 `D:\Work\codex\codex-flightcontrol`。
- 当前远程仓库使用 GitHub：`https://github.com/Lemonray913/codex-flightcontrol.git`。
- MATLAB 的源代码管理面板、MATLAB Git API、Codex 中的 Git 命令都操作同一个 `.git` 仓库；任一端提交后，另一端通过 `git status`、`git log` 或刷新 MATLAB 项目即可看到最新状态。
- 如果用户在 MATLAB 中手动提交，Codex 不会自动实时感知；需要先读取 Git 状态和提交历史，例如 `git status --short --branch`、`git log --oneline -5`。
- 如果 Codex 修改代码并需要提交，先检查工作区状态，只提交本次任务相关文件，避免把用户在 MATLAB 中尚未说明的改动混入同一次提交。
- 不要让 MATLAB 和 Codex 同时执行提交、推送、拉取、合并、切分支或回滚；一端操作完成后，另一端先刷新/检查状态再继续。
- 提交信息使用简短明确的英文或中文均可，建议一类改动一次提交，例如 `Update project agent notes` 或 `补充Git协作说明`。
- 推送前确认远程地址和分支：`git remote -v`、`git status --short --branch`。
- 当前分支沿用现有仓库设置；如需把 `master` 改为 `main`，必须先和用户确认并同步 GitHub 默认分支设置。
- `.slx`、`.docx`、`.pdf`、`.xlsx` 等二进制文件可以提交，但冲突通常难以自动合并；协作时避免多人同时修改同一个 Simulink 模型文件。
