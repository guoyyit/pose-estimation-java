package com.github.guoyyit.poseestimationjava.model;

/**
 * Inertial sample.
 *
 * @param timestampNanos monotonic sensor timestamp in nanoseconds
 * @param gyroZ          yaw rate around vertical axis (rad/s), biased
 * @param accelX         longitudinal acceleration (m/s^2)
 * @param accelY         lateral acceleration (m/s^2)
 */
public record ImuSample(long timestampNanos, double gyroZ, double accelX, double accelY) {
}
