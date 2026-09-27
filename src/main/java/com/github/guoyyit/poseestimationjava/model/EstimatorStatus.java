package com.github.guoyyit.poseestimationjava.model;

/** Lifecycle/health state of the estimator. */
public enum EstimatorStatus {
    /** Filter not yet converged: heading/bias covariance above thresholds. */
    UNCONVERGED,
    /** Filter converged and producing usable output. */
    TRACKING,
    /** Input starved or otherwise degraded; output still published but flagged. */
    DEGRADED
}
