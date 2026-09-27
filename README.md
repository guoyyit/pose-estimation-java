# 本地位姿估计与自诊断服务

面向移动平台的一套**本地可运行**的二维位姿估计与自诊断能力。输入为异步到达的
轮速与惯性（仅偏航角速度）采样，经本地内存接口喂入；输出统一单调时间轴上的
位姿、速度、协方差与可靠性状态，并对传感器异常进行区分式检测与隔离。

服务基于 Spring Boot 3 / Java 17，无外部依赖、无需联网即可运行与测试。

---

## 1. 状态模型（EKF）

状态向量（5 维）：

```
x = [ X, Y, θ, v, b ]ᵀ
    │  │  │  │  └─ 陀螺零偏 (rad/s)
    │  │  │  └──── 车体速度 (m/s)
    │  │  └─────── 航向角 (rad，归一化到 (-π, π])
    │  └────────── 平面位置 y (m)
    └───────────── 平面位置 x (m)
```

协方差 `P` 为 5×5 对称正定矩阵，更新采用 **Joseph 形式**
`P ← (I−KH)P(I−KH)ᵀ + K R Kᵀ`，数值稳定。

### 运动模型（预测）

```
X' = X + v·cos(θ)·dt
Y' = Y + v·sin(θ)·dt
θ' = θ + (ω − b)·dt        ω 为陀螺读数（零阶保持）
v  速度随机游走；b 零偏随机游走
```

- 轮速给出 `v` 的直接测量（`H = e_v`，测量噪声 `R = wheelNoiseVariance`）。
- 陀螺在**运动段**作为控制输入（其噪声进入 θ 的过程噪声）；
  在**静止段**作为零偏测量（`H = e_b`，测量值即陀螺读数）实现 ZUPT，
  同时施加 `v = 0` 约束，因此静止时位置/航向不漂移、零偏可观测收敛。

### 静止判定（纯数据驱动、顺序无关）

- 合并时间轴上，取“截至当前时刻的最近一个非零轮速时刻”；
- 当前为非零轮速 ⇒ 运动；曾运动后需速度连续低于 `staticSpeedThreshold`
  达 `STATIC_DWELL_NANOS = 200ms` 才回到静止；从未观测到运动时从时间轴
  起点计驻留。该判定只依赖时间轴上的测量集合，与样本到达顺序无关。

### 可用性状态（区分未收敛 / 可用）

- `UNCONVERGED`：`P_bb > biasConvergedVariance` 或 `P_θθ > headingConvergedVariance`；
- `USABLE`：两者均收敛到阈值以内。

默认配置零偏/航向初值方差极小，启动即 `USABLE`；把零偏初值方差调大
（如 `withBiasInit(0, 1.0)`）可演示在线“未收敛 → 可用”的收敛过程。

> 说明：绝对航向在仅靠轮速 + 单轴陀螺、无外部航向基准时不可观测，
> 系统估计的是相对初始航向的航迹；`initialHeading` 作为外部先验注入。

### 协方差不发散

每个对角元都被夹在 `[covarianceFloor, covarianceCap]`；速度随机游走噪声
只在“含轮速”的时刻按距上一轮速的绝对时长结算，纯陀螺点不重复注入，
保证长时间运行与乱序重放下协方差有界且确定。

---

## 2. 连续时间轴与乱序/回拨处理

- 两路采样各自保存在 `OrderedSampleBuffer` 并始终保持时间升序，支持异步、乱序。
- 每进入一个新采样，估计器从**提交检查点**（时间 C，状态不可变）把窗口内
  两路采样按**时间点分组归并**后整体重处理，重建 `[C, T]` 的确定轨迹。
- 接纳规则（原因均可区分，见 `AnomalyType`）：
  - `DUPLICATE_TIMESTAMP`：同流同时间戳，**拒绝**；
  - `STALE_SAMPLE`：早于 `C − horizonNanos`，无法并入，**拒绝**；
  - `TIMESTAMP_ROLLBACK`：窗口内的迟到/回拨采样，**接纳并入序**，
    由重处理吸收，时间轴保持连续，同时事件保留原因；
  - 正常采样按序接纳。
- 查询时精确事件点直接返回该确定状态，非事件点在相邻节点间线性插值，
  超出末端则沿模型外推。输出对象 `PoseEstimate` 不可变。

---

## 3. 传感器异常检测与隔离

| 类型 | 判据 | 处理 |
| --- | --- | --- |
| `WHEEL_SPEED_JUMP` | 新轮速与上一**有效**轮速之差 > `wheelJumpThreshold` | 丢弃，不更新状态、不入重处理 |
| `GYRO_BIAS_JUMP` | 静止段 `|陀螺读数 − 零偏估计| > biasJumpThreshold` | 丢弃，状态冻结 |
| `INPUT_TIMEOUT` | 查询时刻距最后有效输入 > `inputTimeoutNanos` | 附加在该次输出上，不改变状态 |
| `DUPLICATE_TIMESTAMP` / `STALE_SAMPLE` / `TIMESTAMP_ROLLBACK` | 见上节 | 拒绝或接纳并报告 |

异常采样在**进入缓冲区之前**完成内容预检，被拒样本不会出现在重放集合中，
因此无法污染位姿；异常原因以结构化 `AnomalyEvent(type, timeNanos, detail)`
给出，人类可读且类型可区分。

---

## 4. 配置参数语义（`PoseConfig`）

所有时间单位纳秒、角度弧度、长度米。不可变值对象，通过 `with*` 派生新配置，
服务原子切换引用，**仅作用于切换之后的估计**，已提交状态不跳变。

| 参数 | 含义 |
| --- | --- |
| `processNoiseVelocity` | 速度随机游走，方差增量 (m/s)²/ns |
| `processNoiseBias` | 零偏随机游走，方差增量 (rad/s)²/ns |
| `wheelNoiseVariance` | 轮速测量方差 (m/s)² |
| `gyroNoiseVariance` | 陀螺测量方差 (rad/s)² |
| `initialHeading/X/Y` | 初始航向与位置 |
| `initialBias` / `initialBiasVariance` | 陀螺零偏初值与初始方差 |
| `initialHeadingVariance` | 初始航向方差 |
| `biasConvergedVariance` / `headingConvergedVariance` | 零偏/航向收敛阈值 |
| `horizonNanos` | 重处理时间窗口；更早的迟到样本判 STALE |
| `wheelJumpThreshold` | 轮速突变阈值 (m/s) |
| `biasJumpThreshold` | 零偏跳变阈值 (rad/s) |
| `inputTimeoutNanos` | 无输入超时阈值 (ns) |
| `staticSpeedThreshold` | 静止速度阈值 (m/s) |
| `covarianceCap` / `covarianceFloor` | 协方差对角元饱和上下限 |
| `randomSeed` | 确定性仿真随机种子 |

`application.properties` 前缀 `pose` 提供 `stateFilePath`、`randomSeed`。

---

## 5. 持久化与重启恢复

- `JsonFilePoseStateStore` 将最近检查点（位姿、速度、零偏、5×5 协方差、
  时间戳）以 JSON 落盘，写入采用“临时文件 + 原子 rename”，不会读到半文件。
- 重启构造服务时自动恢复检查点，估计器以该点为起点，后续输出与重启前
  **连续、无位姿跳变**。
- 显式 `POST /pose/persist` 触发落盘（也可按需周期持久化）。

---

## 6. 并发模型与确定性

- `PoseEstimationService` 用单把锁串行化所有输入与查询；一次查询构造的是
  不可变 `PoseEstimate`，读取到的位姿必对应某个确定时间点，不会读到半更新。
- 系统不使用墙钟、不使用随机数做状态决策（超时按查询显式时间判定），
  因此给定相同采样序列与配置，输出**逐条一致、可重放**；
  `TrajectorySimulator` 在固定 `randomSeed` 下噪声序列确定。

---

## 7. HTTP 接口（本地）

| 方法 | 路径 | 说明 |
| --- | --- | --- |
| POST | `/pose/input/wheel` | 喂入轮速 `{timeNanos, speed}`，返回是否接纳及异常原因 |
| POST | `/pose/input/imu` | 喂入陀螺 `{timeNanos, gyroZ}` |
| GET | `/pose/latest` | 最近确定时间点输出 |
| GET | `/pose/query?timeNanos=` | 指定时间点输出（插值/外推） |
| GET | `/pose/anomalies` | 异常事件历史 |
| POST | `/pose/persist` | 持久化最近状态 |

启动：`mvn spring-boot:run`。

---

## 8. 自动化测试覆盖

`mvn test` 运行 21 个测试，覆盖：

- 乱序/重复/回拨/过期采样（`OrderedSampleBufferTest`）；
- 乱序与按序到达逐条一致、运动学、时间轴单调（`PoseEstimatorOrderingTest`）；
- 静止不漂移、零偏在线收敛、长时协方差有界（`PoseEstimatorStaticTest`、`BiasDriftTest`）；
- 轮速突变、零偏跳变、输入超时（`PoseServiceAnomalyTest`）；
- 固定种子位级确定性（`DeterminismTest`）；
- 重启恢复连续无跳变（`PoseRestartRecoveryTest`）；
- 高并发输入与查询一致（`PoseConcurrencyTest`）；
- 配置热更新只作用于后续（`ConfigUpdateTest`）。
