package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.diagnostics.AnomalyDetector;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.EstimateStatus;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.PoseState;
import com.github.guoyyit.poseestimationjava.model.WheelSample;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * 位姿估计器：将异步到达的轮速与惯性采样融合到统一单调时间轴。
 *
 * <h2>状态模型（扩展卡尔曼滤波）</h2>
 * <p>状态向量 x = [x, y, θ, v, b]ᵀ：平面位置、航向、车体速度、陀螺零偏；
 * 协方差 P 为 5×5。运动模型：</p>
 * <pre>
 *   x' = x + v·cos(θ)·dt
 *   y' = y + v·sin(θ)·dt
 *   θ' = θ + (ω − b)·dt          （ω 为陀螺读数）
 *   v  按随机游走；b 按随机游走
 * </pre>
 * <p>轮速给出 v 的直接测量；陀螺在运动段作为控制输入（噪声进入 θ），
 * 静止段则作为 b 的测量（ZUPT），并施加 v=0 约束。</p>
 *
 * <h2>连续时间轴与乱序/回拨</h2>
 * <p>两路采样各自按时间入序（{@link OrderedSampleBuffer}）。每次进入新采样，
 * 从提交检查点（时间 C，状态已不可变）开始，将窗口内两路采样按时间归并重处理，
 * 得到直到最新采样时间 T 的确定轨迹。晚到但落在窗口内的回拨采样被重处理吸收，
 * 输出仍单调连续；窗口之外的采样被拒绝（STALE_SAMPLE）。</p>
 *
 * <h2>静止处理与零偏收敛</h2>
 * <p>|v| 持续低于静止阈值达到驻留时间后进入静止：位置/航向冻结，陀螺零偏通过
 * y = b 的观测（测量值即陀螺读数）更新并收敛，同时施加 v=0 更新，
 * 因此静止状态不漂移。静止段陀螺读数相对零偏跳变超阈值时拒绝该采样
 * （GYRO_BIAS_JUMP）。</p>
 *
 * <h2>可用性</h2>
 * <p>P_bb ≤ biasConvergedVariance 且 P_θθ ≤ headingConvergedVariance 时为
 * {@link EstimateStatus#USABLE}，否则 {@link EstimateStatus#UNCONVERGED}。
 * 协方差每个对角元均被夹在 [covarianceFloor, covarianceCap]，长时间运行不发散。</p>
 *
 * <h2>确定性与并发</h2>
 * <p>不使用墙钟、不使用随机数，相同采样序列与配置产生逐条一致输出；
 * 方法本身不持锁，由 {@code PoseEstimationService} 的单把锁串行化输入与查询。</p>
 */
public final class PoseEstimator {

    static final int X = 0, Y = 1, TH = 2, V = 3, B = 4, N = 5;
    /** 进入静止所需的低速驻留时间（纳秒）。 */
    private static final long STATIC_DWELL_NANOS = 200_000_000L;

    private PoseConfig config;
    private final OrderedSampleBuffer buffer;
    private final AnomalyDetector detector;

    /** 提交检查点：时间 C 处已确定、不可变的状态。 */
    private final PoseState committed = new PoseState();
    private boolean initialized = false;
    /** 检查点处的突变检测基准（上一有效轮速）。 */
    private boolean checkpointHasWheel = false;
    private double checkpointWheelSpeed = 0.0;
    private long checkpointWheelTime = Long.MIN_VALUE;
    /** 检查点处的静止判定基准（最近一次非静止时间）。 */
    private long checkpointLastMovingTime = Long.MIN_VALUE;
    /** 检查点处上一轮速时间，用于跨窗口的 v 过程噪声结算。 */
    private long checkpointLastWheelForNoise = Long.MIN_VALUE;
    /** 检查点是否已被“推进/锁定”；锁定前锚点可随更早采样回退。 */
    private boolean anchorLocked = false;

    /** 最近一次重处理的时间轴（首点在 C，末点在 T），供查询插值。 */
    private final List<TimedState> trajectory = new ArrayList<>();
    private PoseState head;
    private long lastInputTime = Long.MIN_VALUE;
    private double lastOmega = 0.0;
    /** 重处理过程中产生的事件（每次重放重建，天然去重）。 */
    private final List<AnomalyEvent> windowEvents = new ArrayList<>();

    public PoseEstimator(PoseConfig config) {
        this.config = config;
        this.buffer = new OrderedSampleBuffer(config.horizonNanos);
        this.detector = new AnomalyDetector(config);
        this.head = freshState(config);
    }

    public void updateConfig(PoseConfig config) {
        this.config = config;
        this.buffer.setHorizonNanos(config.horizonNanos);
        this.detector.updateConfig(config);
    }

    /** 初始/重置状态：应用配置中的初值与初始方差。 */
    private static PoseState freshState(PoseConfig c) {
        PoseState s = new PoseState();
        s.x = c.initialX;
        s.y = c.initialY;
        s.theta = normalize(c.initialHeading);
        s.v = 0.0;
        s.bias = c.initialBias;
        s.timeNanos = Long.MIN_VALUE;
        s.covariance[X * N + X] = clampCov(c, 1.0e-6);
        s.covariance[Y * N + Y] = clampCov(c, 1.0e-6);
        s.covariance[TH * N + TH] = clampCov(c, c.initialHeadingVariance);
        s.covariance[V * N + V] = clampCov(c, 1.0e-6);
        s.covariance[B * N + B] = clampCov(c, c.initialBiasVariance);
        return s;
    }

    public Optional<AnomalyEvent> ingest(WheelSample sample) {
        OrderedSampleBuffer.AddOutcome outcome = buffer.addWheel(sample);
        if (!outcome.isAccepted()) {
            // DUPLICATE / STALE：丢弃
            return Optional.ofNullable(outcome.event());
        }
        // 内容预检：以“时间严格更早的上一有效轮速”为基准判断突变。
        // 检查点基准（窗口外最后一个有效轮速）优先，否则取缓冲中更早的轮速。
        Double baseSpeed = null;
        long baseTime = sample.timeNanos();
        List<WheelSample> wheels = buffer.wheels();
        for (int i = wheels.size() - 1; i >= 0; i--) {
            WheelSample candidate = wheels.get(i);
            if (candidate.timeNanos() < sample.timeNanos()) {
                baseSpeed = candidate.speed();
                baseTime = candidate.timeNanos();
                break;
            }
        }
        if ((baseSpeed == null || baseTime >= sample.timeNanos())
                && checkpointHasWheel && checkpointWheelTime < sample.timeNanos()) {
            baseSpeed = checkpointWheelSpeed;
            baseTime = checkpointWheelTime;
        }
        if (baseSpeed != null) {
            detector.beginRun();
            detector.acceptWheel(baseTime, baseSpeed);
            Optional<AnomalyEvent> jump =
                    detector.inspectWheel(sample.timeNanos(), sample.speed());
            if (jump.isPresent()) {
                // 不入缓冲、不重处理：异常采样无法污染时间轴
                buffer.removeWheelAt(sample.timeNanos());
                return jump;
            }
        }
        reprocess();
        // 接纳的迟到/回拨采样：时间轴已吸收，但事件原因仍返回给调用方
        return Optional.ofNullable(outcome.event());
    }

    public Optional<AnomalyEvent> ingest(ImuSample sample) {
        OrderedSampleBuffer.AddOutcome outcome = buffer.addImu(sample);
        if (!outcome.isAccepted()) {
            return Optional.ofNullable(outcome.event());
        }
        // 静止段零偏跳变预检：静止判据与重放完全一致（数据驱动、顺序无关）
        if (initialized && sample.timeNanos() >= head.timeNanos
                && isStationaryAt(sample.timeNanos())) {
            Optional<AnomalyEvent> biasJump = detector.inspectGyroStationary(
                    sample.timeNanos(), sample.gyroZ(), head.bias);
            if (biasJump.isPresent()) {
                buffer.removeImuAt(sample.timeNanos());
                return biasJump;
            }
        }
        reprocess();
        // 接纳的迟到/回拨采样：时间轴已吸收，但事件原因仍返回给调用方
        return Optional.ofNullable(outcome.event());
    }

    /**
     * 从检查点把窗口内两路采样按时间归并重处理，重建 [C, T] 时间轴。
     * 起点状态固定、采样序列固定，因此结果确定且可重复。
     */
    private void reprocess() {
        Optional<Long> maxT = buffer.maxAcceptedTime();
        if (maxT.isEmpty()) {
            return;
        }
        if (!initialized) {
            // 首个样本到达：以配置初值在“最早采样时刻”建立初始锚点，
            // 但不裁剪缓冲；随后进入正常归并，让首个样本被真正处理。
            long t0 = buffer.minAcceptedTime().orElse(maxT.get());
            anchorAt(t0);
            initialized = true;
            lastInputTime = t0;
            // 不 return：继续执行下方归并，处理已在缓冲中的采样
        } else if (!anchorLocked) {
            // 检查点尚未推进：更早的迟到采样可把锚点回退，保证最终时间轴与按序一致
            long earliest = buffer.minAcceptedTime().orElse(committed.timeNanos);
            if (earliest < committed.timeNanos) {
                anchorAt(earliest);
            }
        }

        List<WheelSample> wheels = buffer.wheels();
        List<ImuSample> imus = buffer.imus();

        PoseState cur = copy(committed);
        double omega = lastOmegaAtCheckpoint();
        long lastMovingTime = checkpointLastMovingTime;
        // 窗口内最后一个有效轮速（重放后用于刷新检查点基准）
        boolean runHasWheel = checkpointHasWheel;
        double runWheelSpeed = checkpointWheelSpeed;
        long runWheelTime = checkpointWheelTime;
        // v 噪声结算基准：上一轮速时间。取检查点记录、重放起点、
        // 以及窗口内不晚于起点的轮速中的最晚者（顺序无关）。
        long lastWheelForNoise = Math.max(checkpointLastWheelForNoise, cur.timeNanos);
        for (WheelSample w0 : wheels) {
            if (w0.timeNanos() <= cur.timeNanos) {
                lastWheelForNoise = Math.max(lastWheelForNoise, w0.timeNanos());
            } else {
                break;
            }
        }

        List<TimedState> rebuilt = new ArrayList<>();
        rebuilt.add(new TimedState(cur.timeNanos, copy(cur)));
        windowEvents.clear();

        int wi = 0, ii = 0;
        long origin = rebuilt.get(0).t;
        while (wi < wheels.size() || ii < imus.size()) {
            long tw = wi < wheels.size() ? wheels.get(wi).timeNanos() : Long.MAX_VALUE;
            long ti = ii < imus.size() ? imus.get(ii).timeNanos() : Long.MAX_VALUE;
            long t = Math.min(tw, ti);
            if (t < cur.timeNanos) {
                // 早于当前重放位置（锚点回退的极端边界），按游标跳过
                if (tw == t) {
                    wi++;
                }
                if (ti == t) {
                    ii++;
                }
                continue;
            }
            // 收集同一时间点的所有陀螺与轮速（分组与到达顺序无关）
            List<ImuSample> imuGroup = new ArrayList<>();
            List<WheelSample> wheelGroup = new ArrayList<>();
            while (ii < imus.size() && imus.get(ii).timeNanos() == t) {
                imuGroup.add(imus.get(ii++));
            }
            while (wi < wheels.size() && wheels.get(wi).timeNanos() == t) {
                wheelGroup.add(wheels.get(wi++));
            }
            boolean movingNow = wheelGroup.stream()
                    .anyMatch(w0 -> Math.abs(w0.speed()) >= config.staticSpeedThreshold);
            // 静止判定完全由数据时间轴决定（顺序无关）：
            //  结合检查点运动基准与窗口内轮速，找“截至 t 的最近非零轮速时刻”。
            //  - 当前点非零轮速 => 运动；
            //  - 存在历史非零轮速 => 距其驻留 STATIC_DWELL 后才静止；
            //  - 从未观测到任何非零轮速 => 从时间轴起点驻留 STATIC_DWELL 后静止。
            long lastMotionOnAxis = checkpointLastMovingTime;
            for (int k = 0; k < wheels.size() && wheels.get(k).timeNanos() <= t; k++) {
                if (Math.abs(wheels.get(k).speed()) >= config.staticSpeedThreshold) {
                    lastMotionOnAxis = wheels.get(k).timeNanos();
                }
            }
            boolean stationary;
            if (movingNow) {
                stationary = false;
            } else if (lastMotionOnAxis != Long.MIN_VALUE) {
                stationary = t - lastMotionOnAxis >= STATIC_DWELL_NANOS;
            } else {
                stationary = t - origin >= STATIC_DWELL_NANOS;
            }

            // 先做一次时间预测（静止或运动模型），再施加该点全部测量更新。
            // 区间角速度取“不晚于 t 的最近陀螺”，由已排序的 imus 按时间查找，
            // 与样本到达/游标顺序无关，保证乱序重放结果一致。
            // 区间 [cur.timeNanos, t] 的积分角速度取“不晚于区间左端点”的最近陀螺，
            // 即上一点的零阶保持值；当前点陀螺在预测之后才生效（供下一区间）。
            // 由完整已排序序列按时间查找，与样本到达/游标顺序无关。
            double intervalOmega = lastGyroAtOrBefore(imus, cur.timeNanos, omega);
            // v 随机游走只在含轮速的区间结算：结算量 = 距上一轮速的绝对时长；
            // 纯陀螺区间为 0。只依赖时间轴，乱序重放一致。
            long vNoiseDtn = wheelGroup.isEmpty() ? 0L : (t - lastWheelForNoise);
            predict(cur, t, intervalOmega, stationary, vNoiseDtn);
            if (!wheelGroup.isEmpty()) {
                lastWheelForNoise = t;
            }
            if (!imuGroup.isEmpty()) {
                omega = imuGroup.get(imuGroup.size() - 1).gyroZ();
            }
            if (stationary) {
                // 静止：陀螺读数作为零偏测量，并施加 v=0 约束
                for (ImuSample im : imuGroup) {
                    scalarUpdate(cur, B, im.gyroZ(), config.gyroNoiseVariance);
                }
                for (WheelSample w0 : wheelGroup) {
                    scalarUpdate(cur, V, w0.speed(), config.wheelNoiseVariance);
                }
                scalarUpdate(cur, V, 0.0,
                        config.staticSpeedThreshold * config.staticSpeedThreshold);
            } else {
                // 运动：轮速更新速度（区间内有轮速时以更新值为准）
                for (WheelSample w0 : wheelGroup) {
                    scalarUpdate(cur, V, w0.speed(), config.wheelNoiseVariance);
                    runHasWheel = true;
                    runWheelSpeed = w0.speed();
                    runWheelTime = t;
                }
            }
            rebuilt.add(new TimedState(cur.timeNanos, copy(cur)));
        }

        head = cur;
        lastOmega = omega;
        lastInputTime = maxT.get();
        replaceTrajectory(rebuilt);

        // 检查点静止基准：直接由时间轴上最后一个非零轮速时刻决定（顺序无关）
        long lastMotionOnFullAxis = checkpointLastMovingTime;
        for (WheelSample w0 : wheels) {
            if (Math.abs(w0.speed()) >= config.staticSpeedThreshold) {
                lastMotionOnFullAxis = Math.max(lastMotionOnFullAxis, w0.timeNanos());
            }
        }
        lastMovingTime = lastMotionOnFullAxis;

        // 检查点推进：保留最近 horizon 的原始采样与对应状态
        long cutoff = maxT.get() - config.horizonNanos;
        if (cutoff > committed.timeNanos) {
            PoseState atCutoff = stateAt(cutoff);
            checkpointHasWheel = runHasWheel;
            checkpointWheelSpeed = runWheelSpeed;
            checkpointWheelTime = runWheelTime;
            checkpointLastMovingTime = lastMovingTime;
            checkpointLastWheelForNoise = lastWheelForNoise;
            copy(atCutoff, committed);
            anchorLocked = true;
            buffer.prune(cutoff);
            // 裁剪时间轴到新检查点
            trajectory.removeIf(p -> p.t < cutoff);
            if (trajectory.isEmpty() || trajectory.get(0).t != cutoff) {
                trajectory.add(0, new TimedState(cutoff, copy(committed)));
            }
        }
    }

    private double lastOmegaAtCheckpoint() {
        return lastOmega;
    }

    /** 以配置初值在给定时间建立初始锚点。 */
    private void anchorAt(long t0) {
        PoseState s = freshState(config);
        s.timeNanos = t0;
        copy(s, committed);
        head = copy(s);
        // 静止完全由数据识别（见 isStationaryAt）：纯静止数据会 ZUPT，
        // 含运动数据的前导陀螺不会被错误地当作零偏，且结论顺序无关。
        trajectory.clear();
        trajectory.add(new TimedState(t0, copy(s)));
    }

    private void replaceTrajectory(List<TimedState> rebuilt) {
        trajectory.clear();
        trajectory.addAll(rebuilt);
    }

    /**
     * 数据驱动的静止判定（顺序无关）：结合检查点运动基准与缓冲轮速，
     * 找截至 {@code t} 的最近非零轮速时刻；从未运动则从时间轴起点计驻留。
     */
    private boolean isStationaryAt(long t) {
        long lastMotion = checkpointLastMovingTime;
        for (WheelSample w0 : buffer.wheels()) {
            if (w0.timeNanos() > t) {
                break;
            }
            if (Math.abs(w0.speed()) >= config.staticSpeedThreshold) {
                lastMotion = w0.timeNanos();
            }
        }
        if (lastMotion != Long.MIN_VALUE) {
            return t - lastMotion >= STATIC_DWELL_NANOS;
        }
        long origin = trajectory.isEmpty() ? t : trajectory.get(0).t;
        return t - origin >= STATIC_DWELL_NANOS;
    }

    /**
     * 在已排序陀螺序列中二分查找时间不晚于 t 的最近读数；不存在时返回 fallback。
     */
    private double lastGyroAtOrBefore(List<ImuSample> sorted, long t, double fallback) {
        int lo = 0;
        int hi = sorted.size();
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (sorted.get(mid).timeNanos() <= t) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo == 0 ? fallback : sorted.get(lo - 1).gyroZ();
    }

    /**
     * EKF 预测：把状态从 s.timeNanos 推进到 target。
     *
     * @param omega      该区间采用的陀螺零阶保持值
     * @param stationary 静止模式：均值冻结，仅零偏随机游走
     * @param vNoiseDtn  v 随机游走噪声的积分时长（纳秒）。速度由轮速锚定：
     *                   仅在“含轮速”的区间按距上一轮速时刻的绝对时长一次性结算，
     *                   纯陀螺区间为 0。该时长只依赖时间轴，不依赖区间切片，
     *                   因而乱序/迟到重放结果完全一致。
     */
    private void predict(PoseState s, long target, double omega, boolean stationary,
                         long vNoiseDtn) {
        double dt = (target - s.timeNanos) * 1.0e-9;
        if (dt <= 0) {
            s.timeNanos = target;
            return;
        }
        double[][] F = identity();
        double[][] Q = zeroMatrix();
        if (stationary) {
            // 均值不动；零偏仍有缓慢随机游走
            Q[B][B] = config.processNoiseBias * (target - s.timeNanos);
        } else {
            double th = s.theta;
            double c = Math.cos(th);
            double sn = Math.sin(th);
            s.x += s.v * c * dt;
            s.y += s.v * sn * dt;
            s.theta = normalize(th + (omega - s.bias) * dt);
            F[X][Y] = 0;
            F[X][V] = c * dt;
            F[X][TH] = -s.v * sn * dt;
            F[Y][V] = sn * dt;
            F[Y][TH] = s.v * c * dt;
            F[TH][B] = -dt;
            long dtn = target - s.timeNanos;
            Q[V][V] = config.processNoiseVelocity * Math.max(0L, vNoiseDtn);
            Q[B][B] = config.processNoiseBias * dtn;
            Q[TH][TH] = config.gyroNoiseVariance * dt;
        }
        s.timeNanos = target;
        writeCov(s, add(mult3(F, matrixFromCov(s.covariance), transpose(F)), Q));
        clampDiagonals(s);
    }

    /** 标量卡尔曼更新（Joseph 形式，数值稳定）。 */
    private void scalarUpdate(PoseState s, int hIndex, double z, double r) {
        double[][] P = matrixFromCov(s.covariance);
        double[] x = stateVector(s);
        double hx = x[hIndex];
        double innov = z - hx;
        double sVar = P[hIndex][hIndex] + r;
        double[] K = new double[N];
        for (int i = 0; i < N; i++) {
            K[i] = P[i][hIndex] / sVar;
        }
        for (int i = 0; i < N; i++) {
            x[i] += K[i] * innov;
        }
        double[][] IKH = identity();
        for (int i = 0; i < N; i++) {
            IKH[i][hIndex] -= K[i];
        }
        double[][] joseph = add(
                mult3(IKH, P, transpose(IKH)),
                scale(outer(K, K), r));
        writeCov(s, joseph);
        writeStateVector(s, x);
        s.theta = normalize(s.theta);
        clampDiagonals(s);
    }

    /** 在已确定时间轴上取任意时刻状态：区间内插值，超出末端则沿模型外推。 */
    private PoseState stateAt(long t) {
        if (trajectory.isEmpty()) {
            return copy(head);
        }
        TimedState first = trajectory.get(0);
        TimedState last = trajectory.get(trajectory.size() - 1);
        if (t <= first.t) {
            return copy(first.s);
        }
        if (t >= last.t) {
            PoseState s = copy(last.s);
            boolean stationary = isStationaryAt(t);
            // 查询外推（末端之后）没有新轮速，v 噪声按外推时长连续结算
            predict(s, t, lastOmega, stationary, t - last.t);
            return s;
        }
        for (int i = 0; i < trajectory.size(); i++) {
            TimedState node = trajectory.get(i);
            if (node.t == t) {
                // 精确事件点：直接返回该确定状态，不插值
                return copy(node.s);
            }
        }
        for (int i = 0; i + 1 < trajectory.size(); i++) {
            TimedState a = trajectory.get(i);
            TimedState b = trajectory.get(i + 1);
            if (t > a.t && t < b.t) {
                double alpha = (t - a.t) / (double) (b.t - a.t);
                return interpolate(a, b, alpha);
            }
        }
        return copy(last.s);
    }

    private PoseState interpolate(TimedState a, TimedState b, double alpha) {
        PoseState s = new PoseState();
        s.timeNanos = a.t + (long) (alpha * (b.t - a.t));
        s.x = a.s.x + (b.s.x - a.s.x) * alpha;
        s.y = a.s.y + (b.s.y - a.s.y) * alpha;
        double dTheta = normalize(b.s.theta - a.s.theta);
        s.theta = normalize(a.s.theta + dTheta * alpha);
        s.v = a.s.v + (b.s.v - a.s.v) * alpha;
        s.bias = a.s.bias + (b.s.bias - a.s.bias) * alpha;
        double[][] Pa = matrixFromCov(a.s.covariance);
        double[][] Pb = matrixFromCov(b.s.covariance);
        double[][] P = add(scale(Pa, 1.0 - alpha), scale(Pb, alpha));
        writeCov(s, P);
        clampDiagonals(s);
        return s;
    }

    /**
     * 查询某个确定时间点的输出；返回对象不可变。
     * 调用方（服务层）保证时间点语义并可附加超时等额外事件。
     */
    public PoseEstimate query(long timeNanos, List<AnomalyEvent> extraAnomalies) {
        if (!hasData()) {
            return null;
        }
        PoseState s = stateAt(timeNanos);
        double[] diag = new double[N];
        for (int i = 0; i < N; i++) {
            diag[i] = s.covariance[i * N + i];
        }
        List<AnomalyEvent> events = new ArrayList<>(windowEvents);
        if (extraAnomalies != null) {
            events.addAll(extraAnomalies);
        }
        return new PoseEstimate(s.timeNanos, s.x, s.y, s.theta, s.v,
                statusOf(s), diag, events);
    }

    public PoseState snapshot() {
        if (!hasData()) {
            return null;
        }
        return copy(head);
    }

    /** 包级可见：读取最新时间点零偏均值（测试与自诊断用）。 */
    double currentBiasMean() {
        return hasData() ? head.bias : Double.NaN;
    }

    /** 从持久化状态恢复：以该状态为新检查点，后续输出与其连续、无跳变。 */
    public void restore(PoseState state) {
        PoseState s = copy(state);
        copy(s, committed);
        head = copy(s);
        initialized = true;
        lastInputTime = s.timeNanos;
        checkpointHasWheel = false;
        checkpointLastMovingTime = Math.abs(s.v) >= config.staticSpeedThreshold
                ? s.timeNanos : Long.MIN_VALUE;
        checkpointLastWheelForNoise = s.timeNanos;
        trajectory.clear();
        trajectory.add(new TimedState(s.timeNanos, copy(s)));
        windowEvents.clear();
        buffer.reset();
        buffer.prune(s.timeNanos);
        anchorLocked = true;
    }

    public void reset() {
        buffer.reset();
        committed.timeNanos = Long.MIN_VALUE;
        initialized = false;
        checkpointHasWheel = false;
        checkpointLastMovingTime = Long.MIN_VALUE;
        anchorLocked = false;
        trajectory.clear();
        windowEvents.clear();
        head = freshState(config);
        lastInputTime = Long.MIN_VALUE;
        lastOmega = 0.0;
    }

    public List<AnomalyEvent> windowEvents() {
        return List.copyOf(windowEvents);
    }

    public long lastInputTime() {
        return lastInputTime;
    }

    public boolean hasData() {
        return initialized;
    }

    public EstimateStatus statusOf(PoseState s) {
        double pb = s.covariance[B * N + B];
        double pt = s.covariance[TH * N + TH];
        return (pb <= config.biasConvergedVariance
                && pt <= config.headingConvergedVariance)
                ? EstimateStatus.USABLE : EstimateStatus.UNCONVERGED;
    }

    // ---------- 矩阵/状态工具 ----------

    private static double[][] identity() {
        double[][] m = new double[N][N];
        for (int i = 0; i < N; i++) {
            m[i][i] = 1.0;
        }
        return m;
    }

    private static double[][] zeroMatrix() {
        return new double[N][N];
    }

    private static double[][] transpose(double[][] a) {
        double[][] r = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                r[j][i] = a[i][j];
            }
        }
        return r;
    }

    private static double[][] mult3(double[][] a, double[][] b, double[][] c) {
        // 计算 a * b * c
        double[][] ab = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                double sum = 0;
                for (int k = 0; k < N; k++) {
                    sum += a[i][k] * b[k][j];
                }
                ab[i][j] = sum;
            }
        }
        double[][] r = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                double sum = 0;
                for (int k = 0; k < N; k++) {
                    sum += ab[i][k] * c[k][j];
                }
                r[i][j] = sum;
            }
        }
        return r;
    }

    private static double[][] add(double[][] a, double[][] b) {
        double[][] r = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                r[i][j] = a[i][j] + b[i][j];
            }
        }
        return r;
    }

    private static double[][] scale(double[][] a, double k) {
        double[][] r = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                r[i][j] = a[i][j] * k;
            }
        }
        return r;
    }

    private static double[][] outer(double[] a, double[] b) {
        double[][] r = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                r[i][j] = a[i] * b[j];
            }
        }
        return r;
    }

    private static double[][] matrixFromCov(double[] flat) {
        double[][] m = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                m[i][j] = flat[i * N + j];
            }
        }
        return m;
    }

    private static void writeCov(PoseState s, double[][] m) {
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                // 强制对称，抑制浮点非对称
                s.covariance[i * N + j] = 0.5 * (m[i][j] + m[j][i]);
            }
        }
    }

    private static double[] stateVector(PoseState s) {
        return new double[] {s.x, s.y, s.theta, s.v, s.bias};
    }

    private static void writeStateVector(PoseState s, double[] x) {
        s.x = x[X];
        s.y = x[Y];
        s.theta = x[TH];
        s.v = x[V];
        s.bias = x[B];
    }

    private static PoseState copy(PoseState from) {
        PoseState s = new PoseState();
        copy(from, s);
        return s;
    }

    private static void copy(PoseState from, PoseState to) {
        to.x = from.x;
        to.y = from.y;
        to.theta = from.theta;
        to.v = from.v;
        to.bias = from.bias;
        to.timeNanos = from.timeNanos;
        System.arraycopy(from.covariance, 0, to.covariance, 0, N * N);
    }

    private void clampDiagonals(PoseState s) {
        for (int i = 0; i < N; i++) {
            s.covariance[i * N + i] = clampCov(config, s.covariance[i * N + i]);
        }
    }

    static double normalize(double angle) {
        double a = angle % (2.0 * Math.PI);
        if (a > Math.PI) {
            a -= 2.0 * Math.PI;
        }
        if (a < -Math.PI) {
            a += 2.0 * Math.PI;
        }
        return a;
    }

    static double clampCov(PoseConfig c, double value) {
        return Math.max(c.covarianceFloor, Math.min(c.covarianceCap, value));
    }

    /** 时间轴上一个确定状态点。 */
    private static class TimedState {
        final long t;
        final PoseState s;

        TimedState(long t, PoseState s) {
            this.t = t;
            this.s = s;
        }
    }
}
