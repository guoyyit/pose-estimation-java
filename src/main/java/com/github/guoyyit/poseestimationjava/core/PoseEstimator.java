package com.github.guoyyit.poseestimationjava.core;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyReason;
import com.github.guoyyit.poseestimationjava.model.EstimatorStatus;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.SensorType;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

/**
 * Extended Kalman filter based pose estimator.
 *
 * <p>State: [x, y, heading, velocity, gyroBias]. Gyro is consumed as a control
 * input; wheel speed provides velocity and (via differential) yaw-rate
 * measurements that make the gyro bias observable. Zero-velocity updates keep
 * a stationary platform from drifting and bound the covariance.</p>
 *
 * <p>Asynchronous samples are reordered in a time-window buffer so the
 * committed output timeline is strictly monotonic; duplicates, late samples
 * and timestamp rollbacks are rejected with distinct anomaly reasons.</p>
 *
 * <p>All public methods are synchronized; readers always observe a fully
 * committed immutable {@link PoseEstimate}.</p>
 */
public class PoseEstimator {

    private static final int N = PoseEstimate.DIM;
    private static final double VAR_FLOOR = 1e-12;
    private static final double MAX_PREDICT_DT_SEC = 10.0;
    private static final long WHEEL_REF_FRESH_NANOS = 500_000_000L;

    private PoseEstimatorConfig config;
    private final SampleReorderBuffer buffer = new SampleReorderBuffer();

    private final double[] x = new double[N];
    private final double[][] p = new double[N][N];

    private long lastCommitTime = -1L;
    private long maxArrivalTime = -1L;
    private long clockNow = -1L;
    private long sequence = 0L;

    private double lastGyroZ;
    private long lastImuTime = -1L;
    private double lastWheelYawRate;
    private long lastWheelTime = -1L;

    /** Recent signed wheel speeds: [timestamp, value]. */
    private final ArrayDeque<double[]> recentWheel = new ArrayDeque<>();
    /** Recent signed gyro-minus-bias indicators: [timestamp, value]. */
    private final ArrayDeque<double[]> recentGyro = new ArrayDeque<>();

    private boolean converged;
    private boolean noInputActive;
    private EstimatorStatus status = EstimatorStatus.UNCONVERGED;

    private final List<AnomalyEvent> pendingAnomalies = new ArrayList<>();

    public PoseEstimator(PoseEstimatorConfig config) {
        this.config = config.copy();
        resetToPrior();
    }

    /** Restore constructor: rebuilds the filter from a persisted snapshot. */
    public PoseEstimator(PoseEstimatorConfig config, PersistedState restored) {
        this.config = config.copy();
        System.arraycopy(restored.state(), 0, x, 0, N);
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                p[i][j] = restored.covariance()[i * N + j];
            }
        }
        this.lastCommitTime = restored.timestampNanos();
        this.maxArrivalTime = restored.timestampNanos();
        this.clockNow = restored.timestampNanos();
        this.converged = restored.converged();
        this.lastGyroZ = restored.lastGyroZ();
        this.lastWheelYawRate = restored.lastWheelYawRate();
        this.lastWheelTime = restored.lastWheelTimestampNanos();
        this.lastImuTime = restored.lastImuTimestampNanos();
        refreshStatus();
    }

    private void resetToPrior() {
        Arrays.fill(x, 0.0);
        x[0] = config.initialX;
        x[1] = config.initialY;
        x[2] = config.initialHeading;
        x[4] = config.initialGyroBias;
        for (double[] row : p) {
            Arrays.fill(row, 0.0);
        }
        p[0][0] = config.initialPositionVariance;
        p[1][1] = config.initialPositionVariance;
        p[2][2] = config.initialHeadingVariance;
        p[3][3] = config.initialVelocityVariance;
        p[4][4] = config.initialGyroBiasVariance;
        converged = false;
        status = EstimatorStatus.UNCONVERGED;
    }

    // ------------------------------------------------------------------
    // Input API
    // ------------------------------------------------------------------

    /** Submits a wheel speed sample; may be out of order within the reorder window. */
    public synchronized void submitWheelSpeed(WheelSpeedSample sample) {
        offer(SensorType.WHEEL, sample.timestampNanos(),
                sample.leftSpeed(), sample.rightSpeed(), 0.0);
    }

    /** Submits an IMU sample; may be out of order within the reorder window. */
    public synchronized void submitImu(ImuSample sample) {
        offer(SensorType.IMU, sample.timestampNanos(),
                sample.gyroZ(), sample.accelX(), sample.accelY());
    }

    private void offer(SensorType type, long ts, double v1, double v2, double v3) {
        if (lastCommitTime >= 0 && ts <= lastCommitTime) {
            AnomalyReason reason = ts <= lastCommitTime - config.reorderWindowNanos
                    ? AnomalyReason.TIMESTAMP_ROLLBACK
                    : AnomalyReason.OUT_OF_ORDER;
            anomaly(ts, type, reason, "sample at " + ts + " already past committed time " + lastCommitTime);
            return;
        }
        boolean buffered = buffer.offer(
                new SampleReorderBuffer.BufferedSample(ts, type, v1, v2, v3, sequence++));
        if (!buffered) {
            anomaly(ts, type, AnomalyReason.DUPLICATE, "duplicate sample at " + ts);
            return;
        }
        maxArrivalTime = Math.max(maxArrivalTime, ts);
        clockNow = Math.max(clockNow, ts);
        if (noInputActive) {
            noInputActive = false;
            refreshStatus();
        }
        drain(clockNow - config.reorderWindowNanos);
    }

    /**
     * Advances the estimator clock. Flushes the reorder buffer up to
     * now - reorderWindow and runs the no-input watchdog.
     */
    public synchronized void onTimer(long nowNanos) {
        clockNow = Math.max(clockNow, nowNanos);
        drain(clockNow - config.reorderWindowNanos);
        if (maxArrivalTime >= 0 && clockNow - maxArrivalTime > config.noInputTimeoutNanos) {
            if (!noInputActive) {
                noInputActive = true;
                anomaly(clockNow, SensorType.NONE, AnomalyReason.NO_INPUT,
                        "no input for " + (clockNow - maxArrivalTime) + " ns");
                refreshStatus();
            }
        }
    }

    /** Processes every buffered sample regardless of the reorder horizon. */
    public synchronized void flush() {
        drain(Long.MAX_VALUE);
    }

    private void drain(long horizonNanos) {
        for (SampleReorderBuffer.BufferedSample s : buffer.drainUpTo(horizonNanos)) {
            process(s);
        }
    }

    // ------------------------------------------------------------------
    // Filter core
    // ------------------------------------------------------------------

    private void process(SampleReorderBuffer.BufferedSample s) {
        if (s.type() == SensorType.IMU) {
            processImu(s);
        } else {
            processWheel(s);
        }
        lastCommitTime = s.timestampNanos();
        clampCovariance();
        updateConvergence();
    }

    /**
     * Anti-divergence safeguard: caps each variance component at its configured
     * ceiling. Pure dead reckoning cannot bound the heading uncertainty without
     * an absolute reference, so the reported covariance saturates instead of
     * growing without limit.
     */
    private void clampCovariance() {
        double[] ceilings = {
                config.maxPositionVariance,
                config.maxPositionVariance,
                config.maxHeadingVariance,
                config.maxVelocityVariance,
                config.maxGyroBiasVariance
        };
        for (int i = 0; i < N; i++) {
            if (p[i][i] > ceilings[i]) {
                p[i][i] = ceilings[i];
            }
        }
    }

    private void processImu(SampleReorderBuffer.BufferedSample s) {
        double gyroZ = s.v1();
        // Gyro bias jump detection against the independent wheel rotation reference.
        if (lastWheelTime >= 0 && s.timestampNanos() - lastWheelTime <= WHEEL_REF_FRESH_NANOS) {
            double deviation = gyroZ - x[4] - lastWheelYawRate;
            double refSigma = Math.sqrt(config.wheelYawRateNoiseVar + config.gyroNoiseVar);
            double gate = Math.max(config.gyroJumpGate, config.innovationGateSigma * refSigma);
            if (Math.abs(deviation) > gate) {
                anomaly(s.timestampNanos(), SensorType.IMU, AnomalyReason.GYRO_BIAS_JUMP,
                        "gyro deviates from wheel reference by " + deviation + " rad/s");
                // Re-level the bias so the jumped stream cannot pollute the pose,
                // and inflate its covariance so the filter re-converges quickly.
                x[4] = gyroZ - lastWheelYawRate;
                p[4][4] += config.initialGyroBiasVariance;
            }
        }
        lastGyroZ = gyroZ;
        lastImuTime = s.timestampNanos();
        predictTo(s.timestampNanos());
        pushRecent(recentGyro, s.timestampNanos(), gyroZ - x[4]);
        applyStationaryUpdatesIfNeeded(s.timestampNanos());
    }

    private void processWheel(SampleReorderBuffer.BufferedSample s) {
        predictTo(s.timestampNanos());
        double vMeas = 0.5 * (s.v1() + s.v2());
        double yawMeas = (s.v2() - s.v1()) / config.trackWidth;
        lastWheelYawRate = yawMeas;
        lastWheelTime = s.timestampNanos();

        // Wheel speed jump gate (absolute + n-sigma).
        double sV = p[3][3] + config.wheelSpeedNoiseVar;
        double gate = Math.max(config.wheelJumpGate, config.innovationGateSigma * Math.sqrt(sV));
        if (Math.abs(vMeas - x[3]) > gate) {
            anomaly(s.timestampNanos(), SensorType.WHEEL, AnomalyReason.WHEEL_SPEED_JUMP,
                    "wheel speed " + vMeas + " vs state " + x[3]);
        } else {
            scalarUpdate(new double[]{0, 0, 0, 1, 0}, vMeas, config.wheelSpeedNoiseVar);
        }

        // Wheel yaw rate makes the gyro bias observable: z = yawMeas - gyro =~ -bg.
        if (lastImuTime >= 0) {
            scalarUpdate(new double[]{0, 0, 0, 0, -1}, yawMeas - lastGyroZ,
                    config.wheelYawRateNoiseVar);
        }

        pushRecent(recentWheel, s.timestampNanos(), vMeas);
        applyStationaryUpdatesIfNeeded(s.timestampNanos());
    }

    /** Constant-turn-rate prediction using the latest gyro as control input. */
    private void predictTo(long ts) {
        if (lastCommitTime < 0) {
            return;
        }
        double dt = (ts - lastCommitTime) * 1e-9;
        if (dt <= 0.0) {
            return;
        }
        dt = Math.min(dt, MAX_PREDICT_DT_SEC);
        double omega = lastGyroZ - x[4];
        double thetaMid = x[2] + 0.5 * omega * dt;
        double cos = Math.cos(thetaMid);
        double sin = Math.sin(thetaMid);
        double v = x[3];

        x[0] += v * cos * dt;
        x[1] += v * sin * dt;
        x[2] = normalizeAngle(x[2] + omega * dt);

        double[][] f = MatrixOps.identity(N);
        f[0][2] = -v * sin * dt;
        f[0][3] = cos * dt;
        f[1][2] = v * cos * dt;
        f[1][3] = sin * dt;
        f[2][4] = -dt;

        double[][] q = new double[N][N];
        q[2][2] = config.gyroNoiseVar * dt;
        q[3][3] = config.velocityRandomWalkVar * dt;
        q[4][4] = config.gyroBiasRandomWalkVar * dt;

        double[][] fp = MatrixOps.multiply(f, p);
        double[][] fpft = MatrixOps.multiply(fp, MatrixOps.transpose(f));
        double[][] next = MatrixOps.add(fpft, q);
        MatrixOps.symmetrize(next);
        MatrixOps.floorDiagonal(next, VAR_FLOOR);
        for (int i = 0; i < N; i++) {
            System.arraycopy(next[i], 0, p[i], 0, N);
        }
    }

    /** Scalar EKF measurement update. */
    private void scalarUpdate(double[] h, double z, double r) {
        double[] ph = new double[N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                ph[i] += p[i][j] * h[j];
            }
        }
        double s = r;
        double hx = 0.0;
        for (int i = 0; i < N; i++) {
            s += h[i] * ph[i];
            hx += h[i] * x[i];
        }
        double innovation = z - hx;
        double[] k = new double[N];
        for (int i = 0; i < N; i++) {
            k[i] = ph[i] / s;
        }
        for (int i = 0; i < N; i++) {
            x[i] += k[i] * innovation;
        }
        // P <- (I - K H) P.
        double[][] updated = new double[N][N];
        for (int i = 0; i < N; i++) {
            for (int j = 0; j < N; j++) {
                double sum = 0.0;
                for (int m = 0; m < N; m++) {
                    sum += h[m] * p[m][j];
                }
                updated[i][j] = p[i][j] - k[i] * sum;
            }
        }
        MatrixOps.symmetrize(updated);
        MatrixOps.floorDiagonal(updated, VAR_FLOOR);
        for (int i = 0; i < N; i++) {
            System.arraycopy(updated[i], 0, p[i], 0, N);
        }
        x[2] = normalizeAngle(x[2]);
    }

    // ------------------------------------------------------------------
    // Stationary handling (ZUPT)
    // ------------------------------------------------------------------

    private void pushRecent(ArrayDeque<double[]> deque, long ts, double value) {
        deque.addLast(new double[]{ts, value});
        long cutoff = ts - config.stationaryWindowNanos;
        while (!deque.isEmpty() && deque.peekFirst()[0] < cutoff) {
            deque.pollFirst();
        }
    }

    /**
     * A window counts as calm when it spans the full configured window and the
     * absolute mean of its signed values is below eps. Using the mean (instead
     * of the max) makes the detector robust to white measurement noise while
     * still catching systematic motion or bias residual.
     */
    private boolean windowCalm(ArrayDeque<double[]> deque, long now, double eps) {
        if (deque.isEmpty() || deque.peekFirst()[0] > now - config.stationaryWindowNanos) {
            return false;
        }
        double sum = 0.0;
        for (double[] e : deque) {
            sum += e[1];
        }
        return Math.abs(sum / deque.size()) <= eps;
    }

    private void applyStationaryUpdatesIfNeeded(long now) {
        boolean calm = windowCalm(recentWheel, now, config.stationaryWheelEps)
                && windowCalm(recentGyro, now, config.stationaryGyroEps);
        if (!calm) {
            return;
        }
        // Zero velocity update.
        scalarUpdate(new double[]{0, 0, 0, 1, 0}, 0.0, config.zuptVelocityVar);
        // Zero yaw-rate update (also anchors the bias while still).
        if (lastImuTime >= 0) {
            scalarUpdate(new double[]{0, 0, 0, 0, -1}, -lastGyroZ, config.wheelYawRateNoiseVar);
        }
        // Heading pseudo-anchor: a still platform does not rotate, which keeps
        // the heading covariance bounded over long stationary periods.
        scalarUpdate(new double[]{0, 0, 1, 0, 0}, x[2], config.stationaryHeadingAnchorVar);
    }

    // ------------------------------------------------------------------
    // Status / output
    // ------------------------------------------------------------------

    private void updateConvergence() {
        if (converged) {
            // Hysteresis: fall back only well above the converge thresholds.
            converged = p[2][2] < config.headingConvergedVariance * config.lostThresholdMultiplier
                    && p[4][4] < config.biasConvergedVariance * config.lostThresholdMultiplier;
        } else {
            converged = p[2][2] < config.headingConvergedVariance
                    && p[4][4] < config.biasConvergedVariance;
        }
        refreshStatus();
    }

    private void refreshStatus() {
        status = noInputActive
                ? EstimatorStatus.DEGRADED
                : (converged ? EstimatorStatus.TRACKING : EstimatorStatus.UNCONVERGED);
    }

    /** Latest committed immutable snapshot, or empty if none committed yet. */
    public synchronized Optional<PoseEstimate> currentEstimate() {
        if (lastCommitTime < 0) {
            return Optional.empty();
        }
        return Optional.of(snapshot());
    }

    private PoseEstimate snapshot() {
        double reliability = 1.0 / (1.0 + p[0][0] + p[1][1] + p[2][2]);
        return new PoseEstimate(lastCommitTime, x[0], x[1], x[2], x[3], x[4],
                MatrixOps.flatten(p), status, reliability);
    }

    /** Anomalies detected since the previous call to this method. */
    public synchronized List<AnomalyEvent> drainAnomalies() {
        List<AnomalyEvent> out = List.copyOf(pendingAnomalies);
        pendingAnomalies.clear();
        return out;
    }

    private void anomaly(long ts, SensorType type, AnomalyReason reason, String detail) {
        pendingAnomalies.add(new AnomalyEvent(ts, type, reason, detail));
    }

    /** Applies a new config to subsequent estimates. */
    public synchronized void updateConfig(PoseEstimatorConfig config) {
        this.config = config.copy();
    }

    public synchronized EstimatorStatus status() {
        return status;
    }

    /** Serializable internal state for restart recovery. */
    public synchronized PersistedState persistableState() {
        return new PersistedState(lastCommitTime, Arrays.copyOf(x, N), MatrixOps.flatten(p),
                converged, lastGyroZ, lastWheelYawRate, lastWheelTime, lastImuTime);
    }

    private static double normalizeAngle(double a) {
        double r = a % (2.0 * Math.PI);
        if (r > Math.PI) {
            r -= 2.0 * Math.PI;
        } else if (r < -Math.PI) {
            r += 2.0 * Math.PI;
        }
        return r;
    }

    /** Plain snapshot of everything needed to resume without a pose jump. */
    public record PersistedState(long timestampNanos,
                                 double[] state,
                                 double[] covariance,
                                 boolean converged,
                                 double lastGyroZ,
                                 double lastWheelYawRate,
                                 long lastWheelTimestampNanos,
                                 long lastImuTimestampNanos) {
    }
}
