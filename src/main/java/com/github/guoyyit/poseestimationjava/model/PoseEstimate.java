package com.github.guoyyit.poseestimationjava.model;

import java.util.List;

/**
 * 时间轴上某一确定时刻的连续输出：位姿、速度、协方差与可靠性状态。
 * 该对象不可变，读取方不会看到半更新状态。
 *
 * @param timeNanos          对应的确定时间点（纳秒）
 * @param x                   平面位置 x（m）
 * @param y                   平面位置 y（m）
 * @param theta               航向（rad）
 * @param speed               车体速度（m/s）
 * @param status              未收敛 / 可用
 * @param covarianceDiagonal  协方差矩阵对角元 [X, Y, θ, v, b]
 * @param activeAnomalies     该输出附带的异常事件（如超时）
 */
public record PoseEstimate(long timeNanos,
                           double x,
                           double y,
                           double theta,
                           double speed,
                           EstimateStatus status,
                           double[] covarianceDiagonal,
                           List<AnomalyEvent> activeAnomalies) {

    public PoseEstimate {
        // 防御性拷贝，保证不可变
        covarianceDiagonal = covarianceDiagonal == null
                ? new double[0] : covarianceDiagonal.clone();
        activeAnomalies = activeAnomalies == null ? List.of() : List.copyOf(activeAnomalies);
    }

    @Override
    public double[] covarianceDiagonal() {
        return covarianceDiagonal.clone();
    }
}
