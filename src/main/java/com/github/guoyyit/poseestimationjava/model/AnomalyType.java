package com.github.guoyyit.poseestimationjava.model;

/**
 * 传感器/数据流异常类型，异常原因必须可区分。
 */
public enum AnomalyType {
    /** 轮速相对上一有效采样发生超出阈值的突变。 */
    WHEEL_SPEED_JUMP,
    /** 静止状态下陀螺零偏估计值发生跳变。 */
    GYRO_BIAS_JUMP,
    /** 两路输入在时间轴上长时间无新数据。 */
    INPUT_TIMEOUT,
    /** 同一数据流出现完全相同时间戳。 */
    DUPLICATE_TIMESTAMP,
    /** 采样时间戳早于该数据流已接受的最大时间戳（时间戳回拨）。 */
    TIMESTAMP_ROLLBACK,
    /** 采样晚于重处理窗口，已无法并入当前时间轴。 */
    STALE_SAMPLE
}
