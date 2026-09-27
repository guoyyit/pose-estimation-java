package com.github.guoyyit.poseestimationjava.model;

/**
 * Differential-drive wheel speed sample.
 *
 * @param timestampNanos monotonic sensor timestamp in nanoseconds
 * @param leftSpeed      left wheel linear speed (m/s)
 * @param rightSpeed     right wheel linear speed (m/s)
 */
public record WheelSpeedSample(long timestampNanos, double leftSpeed, double rightSpeed) {
}
