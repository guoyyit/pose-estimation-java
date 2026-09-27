package com.github.guoyyit.poseestimationjava.diagnostics;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyType;

import java.util.Optional;

/**
 * 传感器异常检测（无状态时间副作用，可确定性重放）。
 *
 * <ul>
 *   <li>轮速突变：新轮速与上一<b>有效</b>轮速之差超过 wheelJumpThreshold，
 *       判定 {@link AnomalyType#WHEEL_SPEED_JUMP}，该采样不参与更新。</li>
 *   <li>惯性零偏跳变：仅在静止段可观测。陀螺读数与当前零偏估计之差超过
 *       biasJumpThreshold，判定 {@link AnomalyType#GYRO_BIAS_JUMP}，该采样被拒绝。</li>
 *   <li>长时间无输入：不依赖墙钟，由服务层在 queryAt(显式时间) 时比较
 *       最后有效输入时间得到 {@link AnomalyType#INPUT_TIMEOUT}，保证同输入同输出。</li>
 * </ul>
 *
 * <p>每次窗口重处理以 {@link #beginRun} 开始，内部状态随采样流确定性重建。</p>
 */
public final class AnomalyDetector {

    private PoseConfig config;
    private boolean hasLastWheel = false;
    private double lastValidWheelSpeed = 0.0;
    private long lastValidWheelTime = Long.MIN_VALUE;

    public AnomalyDetector(PoseConfig config) {
        this.config = config;
    }

    public void updateConfig(PoseConfig config) {
        this.config = config;
    }

    /** 开始一次重处理：清空随流重建的判定状态。 */
    public void beginRun() {
        hasLastWheel = false;
        lastValidWheelSpeed = 0.0;
        lastValidWheelTime = Long.MIN_VALUE;
    }

    /**
     * 检查轮速是否突变；返回非空异常时调用方必须丢弃该采样（不调用 accept）。
     */
    public Optional<AnomalyEvent> inspectWheel(long timeNanos, double speed) {
        if (hasLastWheel) {
            double delta = Math.abs(speed - lastValidWheelSpeed);
            if (delta > config.wheelJumpThreshold) {
                return Optional.of(new AnomalyEvent(AnomalyType.WHEEL_SPEED_JUMP, timeNanos,
                        "wheel speed jump: |" + speed + " - " + lastValidWheelSpeed
                                + "| = " + delta + " > " + config.wheelJumpThreshold));
            }
        }
        return Optional.empty();
    }

    /** 登记一条已被滤波器接受的有效轮速（异常采样不登记，保证基准不被污染）。 */
    public void acceptWheel(long timeNanos, double speed) {
        hasLastWheel = true;
        lastValidWheelSpeed = speed;
        lastValidWheelTime = timeNanos;
    }

    public boolean hasLastWheel() {
        return hasLastWheel;
    }

    public double lastValidWheelSpeed() {
        return lastValidWheelSpeed;
    }

    public long lastValidWheelTime() {
        return lastValidWheelTime;
    }

    /**
     * 静止段零偏跳变检测：|陀螺读数 - 当前零偏估计| 超阈值。
     *
     * @param timeNanos    采样时间
     * @param gyroValue    陀螺角速度读数
     * @param currentBias  当前零偏估计
     */
    public Optional<AnomalyEvent> inspectGyroStationary(long timeNanos, double gyroValue,
                                                        double currentBias) {
        double innovation = Math.abs(gyroValue - currentBias);
        if (innovation > config.biasJumpThreshold) {
            return Optional.of(new AnomalyEvent(AnomalyType.GYRO_BIAS_JUMP, timeNanos,
                    "gyro/bias innovation " + innovation + " > "
                            + config.biasJumpThreshold + " (gyro=" + gyroValue
                            + ", bias=" + currentBias + ") while stationary"));
        }
        return Optional.empty();
    }
}
