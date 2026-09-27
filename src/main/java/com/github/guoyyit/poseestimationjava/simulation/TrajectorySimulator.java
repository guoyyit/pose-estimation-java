package com.github.guoyyit.poseestimationjava.simulation;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.WheelSample;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * 确定性轨迹与噪声模拟器：固定随机种子下，相同参数产生逐条一致的采样。
 *
 * <p>采样间隔固定 20ms（两路同频但仍为独立采样点，服务层按时间归并）。
 * 噪声为零均值高斯：轮速标准差取测量噪声标准差，陀螺标准差取陀螺噪声标准差。
 * 随机数使用 JDK {@link Random}（其 LCG 序列在 JDK 内跨次运行确定），
 * 并按“轮速流、陀螺流”分别独立取数，避免乱序喂入影响噪声序列本身。</p>
 */
public final class TrajectorySimulator {

    /** 采样周期 20ms（纳秒）。 */
    public static final long STEP_NANOS = 20_000_000L;

    private final PoseConfig config;

    public TrajectorySimulator(PoseConfig config) {
        this.config = config;
    }

    private Random newWheelRng(long streamOffset) {
        Random r = new Random(config.randomSeed);
        // 消耗少量固定个数，使两路随机流彼此独立且确定
        for (long i = 0; i < streamOffset; i++) {
            r.nextDouble();
        }
        return r;
    }

    private Random newImuRng(long streamOffset) {
        Random r = new Random(config.randomSeed ^ 0x9E3779B97F4A7C15L);
        for (long i = 0; i < streamOffset; i++) {
            r.nextDouble();
        }
        return r;
    }

    /** 生成静止指定时长的轮速采样（零均值小噪声）。 */
    public List<WheelSample> stationaryWheels(long startNanos, long durationNanos) {
        return constantWheels(startNanos, durationNanos, 0.0, 0L);
    }

    public List<ImuSample> stationaryImus(long startNanos, long durationNanos) {
        return arcImus(startNanos, durationNanos, 0.0, config.initialBias, 0L);
    }

    /** 匀速直线/圆弧：轮速恒定 speed（含噪声）。 */
    public List<WheelSample> arcWheels(long startNanos, long durationNanos, double speed) {
        return constantWheels(startNanos, durationNanos, speed, 0L);
    }

    public List<WheelSample> constantWheels(long startNanos, long durationNanos,
                                            double speed, long streamOffset) {
        Random r = newWheelRng(streamOffset);
        double sigma = Math.sqrt(config.wheelNoiseVariance);
        List<WheelSample> out = new ArrayList<>();
        for (long t = startNanos; t < startNanos + durationNanos; t += STEP_NANOS) {
            out.add(new WheelSample(t, speed + sigma * r.nextGaussian()));
        }
        return out;
    }

    /**
     * 圆弧陀螺采样：真实角速度 yawRate，叠加恒定零偏 gyroBias 与白噪声。
     */
    public List<ImuSample> arcImus(long startNanos, long durationNanos,
                                   double yawRate, double gyroBias) {
        return arcImus(startNanos, durationNanos, yawRate, gyroBias, 0L);
    }

    public List<ImuSample> arcImus(long startNanos, long durationNanos,
                                   double yawRate, double gyroBias, long streamOffset) {
        Random r = newImuRng(streamOffset);
        double sigma = Math.sqrt(config.gyroNoiseVariance);
        List<ImuSample> out = new ArrayList<>();
        for (long t = startNanos; t < startNanos + durationNanos; t += STEP_NANOS) {
            out.add(new ImuSample(t, yawRate + gyroBias + sigma * r.nextGaussian()));
        }
        return out;
    }
}
