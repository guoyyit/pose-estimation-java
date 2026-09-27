package com.github.guoyyit.poseestimationjava.config;

/**
 * 位姿估计可配置参数（噪声方差、零偏初值、时间窗口等）。
 *
 * <p>不可变值对象：通过 {@code with*} 方法派生新配置，{@code PoseEstimationService}
 * 在后续估计中原子切换引用，因此配置变更只作用于提交之后的估计。</p>
 *
 * <h2>参数语义与单位</h2>
 * <ul>
 *   <li>所有时间单位为纳秒，角度为弧度，长度为米。</li>
 *   <li>*Variance 字段为方差（标准差平方）；过程噪声的两个字段单位是
 *       “每纳秒的方差增量”，最终 Q = q * dt。</li>
 *   <li>horizonNanos：重处理窗口。采样按各自数据流缓存；晚于
 *       (已提交时间 - horizon) 的迟到采样可以重处理并入时间轴，更早的判 STALE_SAMPLE。</li>
 *   <li>initialBiasVariance 同时决定启动状态：小于等于 biasConvergedVariance 时，
 *       认为零偏初值（initialBias）可信，启动即为 USABLE。</li>
 * </ul>
 */
public final class PoseConfig {
    /** 过程噪声：速度随机游走，方差增量 (m/s)^2 / ns。 */
    public final double processNoiseVelocity;
    /** 过程噪声：陀螺零偏随机游走，方差增量 (rad/s)^2 / ns。 */
    public final double processNoiseBias;
    /** 轮速测量噪声方差 (m/s)^2。 */
    public final double wheelNoiseVariance;
    /** 陀螺测量噪声方差 (rad/s)^2。 */
    public final double gyroNoiseVariance;
    /** 初始航向 rad。 */
    public final double initialHeading;
    /** 初始位置 x（m）。 */
    public final double initialX;
    /** 初始位置 y（m）。 */
    public final double initialY;
    /** 陀螺零偏初值（rad/s）。 */
    public final double initialBias;
    /** 初始航向方差 (rad)^2。 */
    public final double initialHeadingVariance;
    /** 初始零偏方差 (rad/s)^2；该值不大于收敛阈值时初始即认为零偏已知、可用。 */
    public final double initialBiasVariance;
    /** 零偏收敛方差阈值 (rad/s)^2，P_bb 低于该值认为零偏收敛。 */
    public final double biasConvergedVariance;
    /** 航向收敛方差阈值 (rad)^2，P_θθ 低于该值认为初始航向已收敛。 */
    public final double headingConvergedVariance;
    /** 重处理/有效时间窗口（纳秒）：窗口外的迟到采样判为 STALE_SAMPLE。 */
    public final long horizonNanos;
    /** 轮速突变阈值：相邻有效采样速度差绝对值 (m/s)。 */
    public final double wheelJumpThreshold;
    /** 陀螺零偏跳变阈值 (rad/s)。 */
    public final double biasJumpThreshold;
    /** 输入超时阈值（纳秒）：两路数据合并后无新采样的最长间隔。 */
    public final long inputTimeoutNanos;
    /** 静止判定速度阈值 (m/s)，|v| 低于该值视为静止。 */
    public final double staticSpeedThreshold;
    /** 协方差饱和上限（各对角元），防止长时间运行发散。 */
    public final double covarianceCap;
    /** 协方差下限，防止数值上奇异。 */
    public final double covarianceFloor;
    /** 确定性仿真使用的随机种子。 */
    public final long randomSeed;

    public PoseConfig(double processNoiseVelocity, double processNoiseBias,
                      double wheelNoiseVariance, double gyroNoiseVariance,
                      double initialHeading, double initialX, double initialY,
                      double initialBias, double initialHeadingVariance,
                      double initialBiasVariance, double biasConvergedVariance,
                      double headingConvergedVariance, long horizonNanos,
                      double wheelJumpThreshold, double biasJumpThreshold,
                      long inputTimeoutNanos, double staticSpeedThreshold,
                      double covarianceCap, double covarianceFloor, long randomSeed) {
        this.processNoiseVelocity = processNoiseVelocity;
        this.processNoiseBias = processNoiseBias;
        this.wheelNoiseVariance = wheelNoiseVariance;
        this.gyroNoiseVariance = gyroNoiseVariance;
        this.initialHeading = initialHeading;
        this.initialX = initialX;
        this.initialY = initialY;
        this.initialBias = initialBias;
        this.initialHeadingVariance = initialHeadingVariance;
        this.initialBiasVariance = initialBiasVariance;
        this.biasConvergedVariance = biasConvergedVariance;
        this.headingConvergedVariance = headingConvergedVariance;
        this.horizonNanos = horizonNanos;
        this.wheelJumpThreshold = wheelJumpThreshold;
        this.biasJumpThreshold = biasJumpThreshold;
        this.inputTimeoutNanos = inputTimeoutNanos;
        this.staticSpeedThreshold = staticSpeedThreshold;
        this.covarianceCap = covarianceCap;
        this.covarianceFloor = covarianceFloor;
        this.randomSeed = randomSeed;
    }

    /**
     * 生成默认配置（本地可直接运行）。
     *
     * <p>默认零偏初值已知且方差极小，因此默认场景启动即 USABLE，
     * 同时仍可通过 {@code withInitialBiasVariance} 放大初值以演示在线收敛。</p>
     */
    public static PoseConfig defaults() {
        final double nsPerSec = 1.0e9;
        return new PoseConfig(
                // 速度过程噪声：等效 0.1 (m/s)^2/s，换算为每纳秒
                0.1 / nsPerSec,
                // 零偏随机游走：等效 (0.0005 rad/s)^2 / s
                (2.5e-7) / nsPerSec,
                // 轮速测量标准差 0.02 m/s
                0.0004,
                // 陀螺测量标准差 0.003 rad/s
                9.0e-6,
                // 初始航向 0，方差极小 => 初始航向可用
                0.0, 0.0, 0.0,
                // 零偏初值 0，方差极小 => 初始即收敛
                0.0,
                1.0e-8,
                1.0e-8,
                // 收敛阈值
                1.0e-6,
                1.0e-4,
                // 重处理窗口 2 秒
                2_000_000_000L,
                // 轮速突变阈值 5 m/s（采样间隔典型 20ms，物理加速度不可能达到）
                5.0,
                // 零偏跳变阈值 0.02 rad/s
                0.02,
                // 输入超时 1 秒
                1_000_000_000L,
                // 静止速度阈值 0.02 m/s
                0.02,
                // 协方差对角元上限/下限
                1.0e3,
                1.0e-12,
                42L);
    }

    /** 用新随机种子派生配置（其余不变）。 */
    public PoseConfig withRandomSeed(long seed) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, initialHeading, initialX, initialY, initialBias,
                initialHeadingVariance, initialBiasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, seed);
    }

    /** 用新的零偏初值与初始方差派生配置（用于演示未收敛→收敛）。 */
    public PoseConfig withBiasInit(double bias, double biasVariance) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, initialHeading, initialX, initialY, bias,
                initialHeadingVariance, biasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, randomSeed);
    }

    /** 用新的航向初值与方差派生配置。 */
    public PoseConfig withHeadingInit(double heading, double headingVariance) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, heading, initialX, initialY, initialBias,
                headingVariance, initialBiasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, randomSeed);
    }

    /** 用新的测量/过程噪声方差派生配置。 */
    public PoseConfig withNoise(double wheelNoiseVariance, double gyroNoiseVariance,
                                double processNoiseVelocity, double processNoiseBias) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, initialHeading, initialX, initialY, initialBias,
                initialHeadingVariance, initialBiasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, randomSeed);
    }

    /** 用新的时间窗口/超时（纳秒）派生配置。 */
    public PoseConfig withWindows(long horizonNanos, long inputTimeoutNanos) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, initialHeading, initialX, initialY, initialBias,
                initialHeadingVariance, initialBiasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, randomSeed);
    }

    /** 用新的异常检测阈值派生配置。 */
    public PoseConfig withThresholds(double wheelJumpThreshold, double biasJumpThreshold,
                                     double staticSpeedThreshold) {
        return new PoseConfig(processNoiseVelocity, processNoiseBias, wheelNoiseVariance,
                gyroNoiseVariance, initialHeading, initialX, initialY, initialBias,
                initialHeadingVariance, initialBiasVariance, biasConvergedVariance,
                headingConvergedVariance, horizonNanos, wheelJumpThreshold,
                biasJumpThreshold, inputTimeoutNanos, staticSpeedThreshold,
                covarianceCap, covarianceFloor, randomSeed);
    }
}
