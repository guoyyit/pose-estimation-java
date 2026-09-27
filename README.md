# pose-estimation-java

本地可运行的移动平台位姿估计与自诊断服务。输入为**轮速**与**惯性（IMU）**的异步采样，
输出连续时间轴上的**位姿、速度与可靠性指标**。所有输入通过本地内存接口模拟
（`PoseEstimatorService` 方法调用），并附带一层 REST 适配器便于手工验证。

## 状态模型

估计器为核心 EKF（`core/PoseEstimator`），状态向量 5 维：

| 分量 | 含义 | 单位 |
|---|---|---|
| `x, y` | 平面位置 | m |
| `heading` | 航向角（全局系，右旋） | rad |
| `velocity` | 前向线速度 | m/s |
| `gyroBias` | 陀螺 Z 轴零偏 | rad/s |

- **过程模型**：陀螺作为控制输入做恒定转弯率积分（中点航向），速度与零偏为随机游走。
- **轮速量测**：左右轮均值 → `velocity` 量测；左右轮差/轮距 → 横摆角速度量测，
  与陀螺组合后使 `gyroBias` 可观测（`z = yawWheel - gyro ≈ -bias`）。
- **零速更新（ZUPT）**：静止判定时施加 `v = 0`、零横摆角与航向伪锚定量测，
  保证静止不漂移、长时运行协方差不发散。
- **航向可观测性说明**：轮速+IMU 无法观测绝对航向，滤波器维护配置先验并约束其
  不确定度；"收敛"定义为航向/零偏方差低于阈值（见下）。

输出 `PoseEstimate` 为**不可变快照**：时间戳、位姿、速度、零偏、5x5 协方差、
状态（`UNCONVERGED` / `TRACKING` / `DEGRADED`）与可靠性 `reliability = 1/(1+Pxx+Pyy+Pθθ)`。
并发读取只会看到某次完整提交，不存在半更新状态。

## 时间轴与异步输入

- 采样可异步、乱序到达；`SampleReorderBuffer` 在 `reorderWindowNanos` 窗口内按
  `(时间戳, 传感器类型, 到达序号)` 全序重排，输出时间轴严格单调。
- 重复样本（同类型同时间戳）丢弃并上报 `DUPLICATE`；晚于已提交时间的样本按
  `OUT_OF_ORDER`（窗口级）或 `TIMESTAMP_ROLLBACK`（严重回拨）拒绝，均不破坏连续性。
- `onTimer(now)` 推进内部时钟：冲刷重排缓冲并运行无输入看门狗
  （`noInputTimeoutNanos` 超时 → `DEGRADED` + `NO_INPUT` 事件，恢复输入后自动回到
  `TRACKING`）。Spring 部署下由 `PoseEstimatorClock` 以 100ms 周期驱动。

## 异常检测（异常采样不污染位姿）

| 原因 | 判定 | 处理 |
|---|---|---|
| `WHEEL_SPEED_JUMP` | 轮速新息超过 `max(wheelJumpGate, nσ·√S)` | 拒绝该次量测，仅提交预测 |
| `GYRO_BIAS_JUMP` | 陀螺与轮速横摆参考偏差超过 `max(gyroJumpGate, nσ·√(R_yaw+R_gyro))` | 零偏立即重调平并膨胀其方差，陀螺流不丢失、位姿不受污染 |
| `NO_INPUT` | 超过 `noInputTimeoutNanos` 无任何输入 | 状态置 `DEGRADED`，恢复后自动清除 |
| `OUT_OF_ORDER` / `TIMESTAMP_ROLLBACK` / `DUPLICATE` | 到达时间违反单调性约束 | 丢弃样本，输出保持连续 |

异常通过 `drainAnomalies()`（或 `GET /api/pose/anomalies`）按可区分原因读取。

## 配置参数语义（`PoseEstimatorConfig`）

| 参数 | 语义 | 默认 |
|---|---|---|
| `initialX/Y/Heading` + `initial*Variance` | 初始位姿先验与方差 | 0 / 1.0 |
| `initialGyroBias` / `initialGyroBiasVariance` | 零偏初值（rad/s）与其方差 | 0 / 0.5 |
| `gyroNoiseVar` | 陀螺噪声功率谱密度（每秒方差） | 1e-3 |
| `wheelSpeedNoiseVar` | 轮速量测噪声方差 (m²/s²) | 1e-3 |
| `wheelYawRateNoiseVar` | 轮速横摆量测噪声方差 | 5e-3 |
| `velocityRandomWalkVar` / `gyroBiasRandomWalkVar` | 速度/零偏随机游走（每秒方差） | 1e-2 / 1e-6 |
| `zuptVelocityVar` / `stationaryHeadingAnchorVar` | ZUPT 与航向锚定量测方差 | 1e-4 |
| `trackWidth` | 轮距 (m) | 0.5 |
| `reorderWindowNanos` | 乱序重排窗口 | 200ms |
| `stationaryWindowNanos` | 静止判定滑窗（窗口内**均值**低于阈值判静止，抗白噪声） | 500ms |
| `noInputTimeoutNanos` | 无输入看门狗超时 | 2s |
| `stationaryWheelEps` / `stationaryGyroEps` | 静止判定均值阈值 | 0.05 / 0.02 |
| `wheelJumpGate` / `gyroJumpGate` / `innovationGateSigma` | 跳变绝对门限 / nσ 系数 | 1.0 / 0.3 / 6 |
| `headingConvergedVariance` / `biasConvergedVariance` | 收敛阈值（→ `TRACKING`） | 0.01 / 0.001 |
| `lostThresholdMultiplier` | 收敛迟滞：超过阈值×倍数才退回 `UNCONVERGED` | 4 |
| `maxPosition/Heading/Velocity/GyroBiasVariance` | 协方差上限钳制（防发散安全网） | 100/10/100/1 |

`updateConfig`（或 `POST /api/pose/config`）深拷贝后作用于**后续**估计，不影响已提交输出。

## 持久化与重启恢复

每次提交后状态（时间戳、状态向量、协方差、收敛标志、最近的陀螺/轮速参考）经
`StateStore` 持久化。`FileStateStore` 采用临时文件 + 原子改名，崩溃不产生半截文件。
重启后从最近快照恢复，恢复出的第一条输出与重启前最后一条**完全一致**（无位姿跳变），
后续输入在恢复的时间轴上继续。状态文件路径由 `pose.state-file` 配置
（默认 `pose-state.txt`）。

## 确定性

估计器本身无随机性；相同输入多重集合（任意到达排列）产生逐条一致的输出序列。
测试中的噪声由固定种子生成。

## 接口一览

内存接口（`PoseEstimatorService`）：`submitWheelSpeed` / `submitImu` / `onTimer` /
`flush` / `currentEstimate` / `drainAnomalies` / `status` / `updateConfig`。

REST：`POST /api/pose/wheel`、`POST /api/pose/imu`、`GET /api/pose/current`、
`GET /api/pose/status`、`GET /api/pose/anomalies`、`POST /api/pose/config`。

## 测试

`mvn test` 覆盖：乱序/重复/时间戳回拨（`OutOfOrderTest`）、零偏估计收敛与漂移跟踪
（`BiasConvergenceTest`）、传感器异常（`SensorAnomalyTest`）、静止不漂移与协方差有界
（`StationaryTest`）、重启恢复与配置热更新（`RestartRecoveryTest`）、并发读写一致性
（`ConcurrencyTest`）、固定种子逐条一致（`DeterminismTest`）。
