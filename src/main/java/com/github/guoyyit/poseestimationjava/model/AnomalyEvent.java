package com.github.guoyyit.poseestimationjava.model;

/**
 * A detected anomaly. Emitted instead of letting a bad sample pollute the state.
 *
 * @param timestampNanos time at which the anomaly was detected (sample or clock time)
 * @param sensor         originating sensor, NONE if not sensor specific
 * @param reason         machine-distinguishable reason
 * @param detail         human readable detail
 */
public record AnomalyEvent(long timestampNanos, SensorType sensor, AnomalyReason reason, String detail) {
}
