package com.github.guoyyit.poseestimationjava.model;

/**
 * 轮速采样（本地内存模拟输入）。
 *
 * @param timeNanos 采样时间戳（单调时钟，纳秒）
 * @param speed     轮速换算的车体线速度，m/s，带符号
 */
public record WheelSample(long timeNanos, double speed) {
}
