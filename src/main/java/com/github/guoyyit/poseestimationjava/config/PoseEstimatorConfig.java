package com.github.guoyyit.poseestimationjava.config;

/**
 * Tunables of the pose estimator. All noise variances expressed "per second"
 * are continuous-time spectral densities multiplied by dt inside the filter.
 *
 * State model: [x, y, heading, velocity, gyroBias].
 */
public class PoseEstimatorConfig {

    // ---- Initial state prior ----
    public double initialX = 0.0;
    public double initialY = 0.0;
    public double initialHeading = 0.0;
    public double initialPositionVariance = 1.0;
    public double initialHeadingVariance = 1.0;
    public double initialVelocityVariance = 1.0;
    /** Gyro bias initial value (rad/s). */
    public double initialGyroBias = 0.0;
    /** Gyro bias initial variance; large means "unknown". */
    public double initialGyroBiasVariance = 0.5;

    // ---- Noise variances ----
    /** Gyro measurement noise variance per second (rad^2/s^2 * s). */
    public double gyroNoiseVar = 1e-3;
    /** Wheel forward-speed measurement noise variance (m^2/s^2). */
    public double wheelSpeedNoiseVar = 1e-3;
    /** Wheel yaw-rate measurement noise variance (rad^2/s^2). */
    public double wheelYawRateNoiseVar = 5e-3;
    /** Velocity random-walk variance per second. */
    public double velocityRandomWalkVar = 1e-2;
    /** Gyro bias random-walk variance per second. */
    public double gyroBiasRandomWalkVar = 1e-6;
    /** Zero-velocity-update measurement variance. */
    public double zuptVelocityVar = 1e-4;
    /** Heading pseudo-anchor variance applied while stationary. */
    public double stationaryHeadingAnchorVar = 1e-4;

    // ---- Geometry ----
    /** Distance between left and right wheels (m). */
    public double trackWidth = 0.5;

    // ---- Time windows (nanoseconds) ----
    /** Out-of-order reordering horizon; samples younger than now-window stay buffered. */
    public long reorderWindowNanos = 200_000_000L;
    /** Sliding window used by the stationary detector. */
    public long stationaryWindowNanos = 500_000_000L;
    /** No-input timeout before the estimator reports DEGRADED/NO_INPUT. */
    public long noInputTimeoutNanos = 2_000_000_000L;

    // ---- Stationary thresholds ----
    public double stationaryWheelEps = 0.05;
    public double stationaryGyroEps = 0.02;

    // ---- Anomaly gates ----
    /** Absolute wheel-speed jump gate (m/s). */
    public double wheelJumpGate = 1.0;
    /** Gyro bias jump gate (rad/s) vs wheel-reported rotation. */
    public double gyroJumpGate = 0.3;
    /** N-sigma innovation gate used in addition to absolute gates. */
    public double innovationGateSigma = 6.0;

    // ---- Convergence thresholds ----
    public double headingConvergedVariance = 0.01;
    public double biasConvergedVariance = 0.001;
    /** Hysteresis: a converged filter falls back only above threshold*multiplier. */
    public double lostThresholdMultiplier = 4.0;

    // ---- Covariance ceilings (anti-divergence safeguard) ----
    public double maxPositionVariance = 100.0;
    public double maxHeadingVariance = 10.0;
    public double maxVelocityVariance = 100.0;
    public double maxGyroBiasVariance = 1.0;

    /** Reasonable defaults for a desktop differential-drive platform. */
    public static PoseEstimatorConfig defaults() {
        return new PoseEstimatorConfig();
    }

    /** Deep copy so a config change never mutates a config already in use. */
    public PoseEstimatorConfig copy() {
        PoseEstimatorConfig c = new PoseEstimatorConfig();
        c.initialX = initialX;
        c.initialY = initialY;
        c.initialHeading = initialHeading;
        c.initialPositionVariance = initialPositionVariance;
        c.initialHeadingVariance = initialHeadingVariance;
        c.initialVelocityVariance = initialVelocityVariance;
        c.initialGyroBias = initialGyroBias;
        c.initialGyroBiasVariance = initialGyroBiasVariance;
        c.gyroNoiseVar = gyroNoiseVar;
        c.wheelSpeedNoiseVar = wheelSpeedNoiseVar;
        c.wheelYawRateNoiseVar = wheelYawRateNoiseVar;
        c.velocityRandomWalkVar = velocityRandomWalkVar;
        c.gyroBiasRandomWalkVar = gyroBiasRandomWalkVar;
        c.zuptVelocityVar = zuptVelocityVar;
        c.stationaryHeadingAnchorVar = stationaryHeadingAnchorVar;
        c.trackWidth = trackWidth;
        c.reorderWindowNanos = reorderWindowNanos;
        c.stationaryWindowNanos = stationaryWindowNanos;
        c.noInputTimeoutNanos = noInputTimeoutNanos;
        c.stationaryWheelEps = stationaryWheelEps;
        c.stationaryGyroEps = stationaryGyroEps;
        c.wheelJumpGate = wheelJumpGate;
        c.gyroJumpGate = gyroJumpGate;
        c.innovationGateSigma = innovationGateSigma;
        c.headingConvergedVariance = headingConvergedVariance;
        c.biasConvergedVariance = biasConvergedVariance;
        c.lostThresholdMultiplier = lostThresholdMultiplier;
        c.maxPositionVariance = maxPositionVariance;
        c.maxHeadingVariance = maxHeadingVariance;
        c.maxVelocityVariance = maxVelocityVariance;
        c.maxGyroBiasVariance = maxGyroBiasVariance;
        return c;
    }
}
