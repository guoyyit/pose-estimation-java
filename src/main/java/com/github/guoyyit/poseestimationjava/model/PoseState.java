package com.github.guoyyit.poseestimationjava.model;

/**
 * 滤波器内部状态快照（也用于持久化恢复）。
 *
 * <p>状态向量 x = [x, y, theta, v, b]：
 * x/y 平面位置(m)，theta 航向(rad)，v 车体速度(m/s)，b 陀螺零偏(rad/s)。</p>
 */
public final class PoseState {
    public double x;
    public double y;
    public double theta;
    public double v;
    public double bias;
    /** 5x5 协方差矩阵，按行存储（25 个元素）。 */
    public final double[] covariance = new double[25];
    public long timeNanos;

    public PoseState() {
    }
}
