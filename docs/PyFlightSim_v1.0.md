# PyFlightSim v1.0 研发路线图与功能基线 (Development Roadmap)

> **版本**：v1.0.2-Stable  
> **发布日期**：2026-09  
> **项目定位**：基于纯 Python 构建的高保真六自由度 (6-DOF) 飞行动力学与全状态闭环飞控仿真框架  
> **文档属性**：v1.0 历史研发里程碑、核心功能基线与踩坑调优记录  
> **下一代演进**：[PyFlightSim v2.0 升级计划与技术规格书](PyFlightSim_v2.0_Upgrade_Plan.md)

---

## 目录
1. [Phase 1: 物理内核构建 (The Physics Kernel)](#phase-1-物理内核构建-the-physics-kernel)
2. [Phase 2: 控制与感知 (GNC & Sensors)](#phase-2-控制与感知-gnc--sensors)
3. [Phase 3: 环境与分析 (Environment & Analysis)](#phase-3-环境与分析-environment--analysis)
4. [Phase 4: 数据驱动重构 (Data-Driven Refactor)](#phase-4-数据驱动重构-data-driven-refactor)
5. [Phase 5: 收尾与发布 (Final Polish & Release)](#phase-5-收尾与发布-final-polish--release)
6. [Phase 6: 核心除Bug与调优 (Bug Fixes & Tuning)](#phase-6-核心除bug与调优-bug-fixes--tuning)
7. [Phase 7: 物理基准与气动耦合终极修复 (v1.0.2 Patch)](#phase-7-物理基准与气动耦合终极修复-v102-patch)
8. [v1.0 总结与向 v2.0 的跨越](#v10-总结与向-v20-的跨越)

---

## Phase 1: 物理内核构建 (The Physics Kernel)

* **状态**：✅ 完成
* **阶段目标**：构建基于牛顿-欧拉力学和四元数的六自由度刚体运动解算引擎。

### 1.1 核心数学库 (MathUtils)
- [x] **四元数运算**：实现 Euler $\leftrightarrow$ Quaternion 互相转换、四元数乘法与共轭运算
- [x] **坐标转换**：机体坐标系 (Body) 与北东地坐标系 (NED) 方向余弦矩阵 (DCM) 双向投影
- [x] **归一化保护**：每步计算执行四元数范数归一化，防止浮点漂移导致旋转矩阵退化

### 1.2 动力学方程 (Dynamics)
- [x] **状态定义 (AircraftState)**：包含位置向量 $\mathbf{P}_{ned}$、速度向量 $\mathbf{V}_b$、姿态四元数 $\mathbf{q}$、角速度向量 $\boldsymbol{\omega}_b$
- [x] **运动学方程 (Kinematics)**：四元数微分方程 $\dot{\mathbf{q}} = \frac{1}{2} \mathbf{q} \otimes [0, \boldsymbol{\omega}]^T$
- [x] **动力学方程 (Dynamics)**：牛顿-欧拉运动学刚体动力学方程解算

### 1.3 数值积分器 (Integrator)
- [x] **RK4 实现**：经典四阶 Runge-Kutta 数值积分推进
- [x] **精度验证**：自由落体与简谐摆动解析解对比测试验证

---

## Phase 2: 控制与感知 (GNC & Sensors)

* **状态**：✅ 完成
* **阶段目标**：建立“传感器测量 - 控制器决策 - 执行机构分配”的数字闭环。

### 2.1 传感器仿真 (Sensors)
- [x] **惯性测量单元 (IMU)**：模拟机体三轴加速度计与三轴角速度陀螺仪
- [x] **大气数据计算机 (AirData)**：解算真空速 (TAS)、气压高度及爬升率
- [x] **卫星导航 (GPS)**：基于平坦地球投影 (Flat Earth Projection) 解算经纬度与地速

### 2.2 控制算法 (Control)
- [x] **工业级 PID 控制器**：集成积分抗饱和 (Anti-windup) 与动态输出钳位 (Clamping)
- [x] **气动混控器 (Mixer)**：支持常规布局 (Standard)、无尾三角翼升降副翼混控 (Delta/Elevon) 及 V 尾混控 (V-Tail)

### 2.3 自动驾驶仪 (Autopilot)
- [x] **纵向通道**：高度控制 (Altitude) $\rightarrow$ 爬升率控制 (Climb Rate) $\rightarrow$ 俯仰姿态环 (Pitch)
- [x] **横侧向通道**：航向保持 (Heading) $\rightarrow$ 滚转角控制 (Roll)
- [x] **速度通道**：自动油门空速保持闭环 (Auto-Throttle)

---

## Phase 3: 环境与分析 (Environment & Analysis)

* **状态**：✅ 完成
* **阶段目标**：引入真实大气物理环境干扰，并建立数据流沉淀与可视化分析体系。

### 3.1 环境模型 (Environment)
- [x] **国际标准大气 (ISA Model)**：随高度自动计算大气温度、气压及空气密度 $\rho(h)$
- [x] **湍流风场模型 (Dryden Turbulence)**：符合军标 MIL-F-8785C 规范的随机连续风扰

### 3.2 数据工程 (Data Engineering)
- [x] **飞行数据记录器 (CSV Logger)**：按步沉淀全状态量、传感器读数及控制输入时序
- [x] **专业可视化工具 (Matplotlib Plotter)**：支持一键导出姿态角、高度、速度及舵偏响应曲线图

---

## Phase 4: 数据驱动重构 (Data-Driven Refactor)

* **状态**：✅ 完成
* **阶段目标**：彻底消除代码层面的硬编码物理常数，全面转向声明式 YAML 配置驱动。

### 4.1 配置系统解耦
- [x] **YAML 加载器 (ConfigLoader)**：支持配置文件解析与默认兜底机制
- [x] **飞机配置 (`configs/aircraft.yaml`)**：质量、转动惯量、几何参考尺寸及气动导数配置化
- [x] **飞控配置 (`configs/autopilot.yaml`)**：各 PID 控制回路增益、限制阈值配置化
- [x] **仿真配置 (`configs/simulation.yaml`)**：积分步长、初始位置、巡航目标点配置化

### 4.2 高级飞行控制特性
- [x] **动压增益调度 (Gain Scheduling)**：控制增益随动压 $\bar{q}$ 自适应缩放，拓宽安全飞行包线
- [x] **偏航阻尼器 (Yaw Damper)**：抑制侧滑振荡，提升飞行平稳度
- [x] **调参优化 (Limousine Mode)**：平滑各通道切换，提升乘坐舒适度与轨迹跟踪精度

---

## Phase 5: 收尾与发布 (Final Polish & Release)

* **状态**：✅ 完成
* **阶段目标**：修补潜在隐患，提升代码鲁棒性与模块独立性，完成开源工程化发布。

### 5.1 核心代码硬化 (Core Hardening)
- [x] **数学稳定性增强**：侧滑角保护 (`np.arctan2`) 与四元数模长防御性归一化
- [x] **积分器安全检查**：限制最大仿真时间步长 $dt$，防止因步长过大导致数值积分发散
- [x] **性能优化**：引入对象池思维降低高频更新下的内存碎片分配

### 5.2 统一数据访问层 (Unified Data Access)
- [x] **统一 Loader 工具**：重构 `tools/loader.py`，集中管理 YAML 读取与 CSV 数据解析
- [x] **路径鲁棒性**：实现基于项目根目录的跨平台绝对路径解析，杜绝 `FileNotFoundError`
- [x] **脚本调用统一**：更新入口脚本 `run_simulation.py` 及全部 `examples/` 示例

### 5.3 工具链升级 (Tooling Upgrade)
- [x] **绘图工具 CLI 化**：重写 `plot_results.py`，支持 `--file` 指定日志与 `--mode euler/quat` 切换
- [x] **多机型算例拓展**：新增高性能喷气机 `configs/su-27.yaml`，验证高机动工况下的稳定性

### 5.4 验证与测试体系
- [x] **配置健全性自检 (Sanity Checks)**：在加载 YAML 参数后增加数值边界合理性检查
- [x] **单元测试**：落地 `test_math3d.py` 与 `test_pid.py` 自动化测试

### 5.5 文档与发布
- [x] **依赖锁定**：沉淀 `requirements.txt` 依赖版本
- [x] **规范文档**：编写专业级中英文 `README.md` 与 `LICENSE` (MIT)

---

## Phase 6: 核心除Bug与调优 (Bug Fixes & Tuning)

* **状态**：✅ 完成
* **阶段目标**：全面清剿 v1.0 初版在边界工况下的潜在问题，沉淀工程排障经验，为高阶模型筑牢地基。

### 6.1 🔴 优先级 1：严重数值与物理缺陷
- [x] **PID 积分饱和 (Integral Windup)**
  * *现象*：大机动或长时间误差积累时，积分项严重超限，改平阶段产生剧烈超调。
  * *解决*：在 `modules/control/pid.py` 实现基于输出饱和的动态钳位 (Dynamic Clamping)。
- [x] **动力学硬编码推力 (Hardcoded Thrust)**
  * *现象*：`forces.py` 曾直接写死大推力常数，导致轻型塞斯纳直接获得火箭般的过载推重比。
  * *解决*：将 `max_thrust` 剥离至 `aircraft.yaml`，并通过数据类 `AeroParams` 统一管理。
- [x] **湍流模型数值发散 (Turbulence Explosion)**
  * *现象*：低空高速时，前向欧拉法导致离散极点溢出单位圆，湍流风速出现指数级数值爆炸。
  * *解决*：推导精确离散传递函数 $e^{-dt/T}$ 代替欧拉近似，并设置有效高度下限。
- [x] **混控器幅值越界 (Mixer Clipping)**
  * *现象*：三角翼/升降副翼混控在滚转与俯仰指令叠加后，物理输出超过 $\pm 1.0$ 的归一化舵面限制。
  * *解决*：在 `mixer.py` 输出前增加矢量截断 (`np.clip`)。

### 6.2 🟠 优先级 2：逻辑与功能瑕疵
- [x] **PID 微分项高频抖动 (D-Term Low-Pass Filter)**
  * *现象*：湍流扰动输入放大微分项噪声，造成操纵舵面高频“打手”颤振。
  * *解决*：在 PID 微分通道串联一阶低通滤波器，滤除高频噪声。
- [x] **偏航阻尼器抑制协调转弯 (Yaw Damper Washout Filter)**
  * *现象*：阻尼器将所有偏航角速度均视为干扰并进行抑制，导致飞机无法进入稳定盘旋。
  * *解决*：在 `autopilot.py` 引入冲刷滤波器 (Washout Filter)，仅抑制高频荷兰滚，放行稳态转弯。
- [x] **高空大气物理边界失效**
  * *现象*：高度超过 11000m 后温度持续线性下降，不符合平流层常温层规律。
  * *解决*：在 `atmosphere.py` 中引入对流层顶以上等温层计算。
- [x] **GPS 极地纬度奇异性**
  * *现象*：纬度逼近 $\pm 90^\circ$ 时因 $\cos(\text{lat}) \to 0$ 导致经度增量计算出现除零发散。
  * *解决*：限制纬度解算有效区间在 $\pm 89.9^\circ$ 以内。
- [x] **大攻角姿态投影**
  * *工程记录*：审查眼镜蛇大迎角机动下的坐标变换逻辑；经实测算法本身无缺陷，将眼镜蛇测试用例解耦为独立进阶场景。

### 6.3 🟡 优先级 3：架构与仿真可用性优化
- [x] **数据记录器内存膨胀 (Streaming CSV Logger)**
  * *现象*：旧版 `DataLogger` 将数万步数据积攒在内存列表中，长航时仿真存在 OOM 风险。
  * *解决*：改为每隔固定步数直接流式刷入磁盘 (Streaming Flush)。
- [x] **协调转弯前馈耦合 (Turn Coordinator)**
  * *现象*：常规转弯仍有微小侧滑角。
  * *解决*：在自动驾驶仪中增加副翼到方向舵的比例前馈通道。
- [x] **仿真初始工况平滑 (Warm-up Phase)**
  * *工程记录*：非配平初始状态启动易带来瞬态俯冲振荡。在引入严格配平算法前，引入开机自稳定“预热期 (Warm-up)”设计。

---

## Phase 7: 物理基准与气动耦合终极修复 (v1.0.2 Patch)

* **状态**：✅ 完成
* **阶段目标**：彻底解决底层物理建模缺失与气流基准断裂问题，实现真正物理自洽的六自由度飞行动力学（原 P0-1 ~ P0-3）。

### 7.1 补全横侧向耦合气动力与力矩 ($C_Y$ 侧向力与 $C_{l,\beta}$ 滚转力矩)
- [x] **缺陷成因**：`forces.py` 侧向力 $F_y$ 被硬编码为 0；滚转力矩缺少机翼上反角效应（二面角效应）导数 $C_{l,\beta} \cdot \beta$ 和方向舵诱导滚转导数 $C_{l,\delta_r} \cdot \delta_r$。
- [x] **修复方案**：
  * 实现完整机体侧向力方程：$F_y = \bar{q} S \cdot (C_{Y,\beta} \cdot \beta + C_{Y,\delta_r} \cdot \delta_r)$
  * 补齐滚转力矩方程：$C_l = C_{l,\beta} \cdot \beta + C_{l,\delta_a} \cdot \delta_a + C_{l,\delta_r} \cdot \delta_r + C_{l,p} \cdot \hat{p}$
  * 配置写入 `aircraft.yaml`：$C_{Y,\beta} = -0.31$, $C_{Y,\delta_r} = 0.187$, $C_{l,\beta} = -0.089$, $C_{l,\delta_r} = 0.010$
- [x] **验收断言**：静态侧滑 $\beta = +5^\circ$ 时验证产生向左侧向力与恢复性左滚力矩；动态模态激发后二阶荷兰滚振荡平稳收敛。

### 7.2 大气数据计算机 (ADC) 全流场风速矢量补偿 (TAS, $\alpha$, $\beta$)
- [x] **缺陷成因**：`AirDataComputer` 仅使用机体对地速度计算 TAS 及攻角 $\alpha$、侧滑角 $\beta$。开启风场或湍流时，控制器感知到的气流角与飞机真实气动受力物理断裂。
- [x] **修复方案**：
  * 接口升级：`update(state, wind_body)` 接收机体坐标系风速向量。
  * 相对风速解算：$\mathbf{V}_{air} = \mathbf{V}_{ground} - \mathbf{V}_{wind} = [u, v, w]^T$。
  * 真实攻角与侧滑角：$\alpha = \arctan2(w, u)$，$\beta = \arcsin(v / V_{tas})$。
- [x] **验收断言**：在 $60\text{ m/s}$ 地速叠加大风下，TAS、$\alpha$、$\beta$ 读数严格与机体相对气流保持数学吻合（逆风 $10\text{ m/s}$ TAS 为 $70\text{ m/s}$；侧风 $5\text{ m/s}$ 侧滑角精确等于 $-4.75^\circ$）。

### 7.3 升降舵气动导数配置化 (消除计算硬编码)
- [x] **缺陷成因**：`forces.py` 内部将升降舵对升力的导数（0.5）和对俯仰力矩的导数（-1.5）直接硬编码写死在计算公式中。
- [x] **修复方案**：在 `AeroParams` 数据类和 `configs/aircraft.yaml` 中增加 `C_L_delta_e` 与 `C_m_delta_e` 声明，从代码逻辑中彻底剥离常数。
- [x] **验收断言**：动态调整 YAML 中导数参数，瞬时力矩精确成比例响应，实现全机型数据驱动无缝切换。

---

## v1.0 总结与向 v2.0 的跨越

PyFlightSim 在经历 Phase 1 到 Phase 7 的完整洗礼后，v1.0.2 已经具备了无懈可击的核心骨架：
* **核心已就绪能力**：完整的 6-DOF 运动解算、气动横侧向力矩真实耦合、级联 PID 自动驾驶、真实 ISA+Dryden 大气环境、全流场风速补偿、YAML 全量数据驱动与流式记录分析。
* **迈向 v2.0 的演进动因**：
  物理基础 Bug 已全部彻底封堵。下一阶段的工作将全面转向更高阶的系统架构突破（如舵机一阶连续动力学、仿真调度器抽象、六自由度动配平求解器、现代控制理论 LQR、FMS 航路规划以及强化学习 Gymnasium 环境）。

👉 **请参阅下一代升级总蓝图**：  
[PyFlightSim v2.0 架构与工程升级技术规格书](PyFlightSim_v2.0_Upgrade_Plan.md)
