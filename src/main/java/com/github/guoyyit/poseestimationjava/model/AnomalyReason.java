package com.github.guoyyit.poseestimationjava.model;

/** Machine-distinguishable reasons why a sample was rejected or the estimator degraded. */
public enum AnomalyReason {
    /** Sample arrived too late to be reordered (older than the committed time). */
    OUT_OF_ORDER,
    /** Exact duplicate of an already buffered sample (same type and timestamp). */
    DUPLICATE,
    /** Timestamp moved backwards beyond the reorder window. */
    TIMESTAMP_ROLLBACK,
    /** Wheel speed innovation exceeded the jump gate. */
    WHEEL_SPEED_JUMP,
    /** Gyro level inconsistent with wheel-reported rotation (bias jump). */
    GYRO_BIAS_JUMP,
    /** No sample arrived within the configured no-input timeout. */
    NO_INPUT
}
