package com.github.guoyyit.poseestimationjava.model;

/**
 * 估计状态：未收敛 / 可用。
 */
public enum EstimateStatus {
    /** 零偏尚未完成收敛，输出仅供观测，不可用于闭环。 */
    UNCONVERGED,
    /** 零偏与初始航向已满足收敛判据，输出可用。 */
    USABLE
}
