package com.github.guoyyit.poseestimationjava.model;

/**
 * 惯性采样（本地内存模拟输入），本系统在二维平面内工作，仅使用偏航角速度。
 *
 * @param timeNanos 采样时间戳（单调时钟，纳秒）
 * @param gyroZ     绕车体 z 轴的角速度，rad/s，逆时针为正
 */
public record ImuSample(long timeNanos, double gyroZ) {
}
