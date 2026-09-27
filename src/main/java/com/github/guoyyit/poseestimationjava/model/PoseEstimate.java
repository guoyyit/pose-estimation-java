package com.github.guoyyit.poseestimationjava.model;

import com.fasterxml.jackson.annotation.JsonAutoDetect;

import java.util.Arrays;

/**
 * Immutable pose snapshot committed at a definite point in time.
 * Readers always observe a fully-formed snapshot, never a half-updated state.
 */
@JsonAutoDetect(fieldVisibility = JsonAutoDetect.Visibility.ANY)
public final class PoseEstimate {

    /** State dimension: [x, y, heading, velocity, gyroBias]. */
    public static final int DIM = 5;

    private final long timestampNanos;
    private final double x;
    private final double y;
    private final double heading;
    private final double velocity;
    private final double gyroBias;
    /** Row-major 5x5 covariance, defensively copied. */
    private final double[] covariance;
    private final EstimatorStatus status;
    /** Reliability score in (0, 1]; 1 means perfectly certain. */
    private final double reliability;

    public PoseEstimate(long timestampNanos,
                        double x,
                        double y,
                        double heading,
                        double velocity,
                        double gyroBias,
                        double[] covariance,
                        EstimatorStatus status,
                        double reliability) {
        if (covariance.length != DIM * DIM) {
            throw new IllegalArgumentException("covariance must be " + (DIM * DIM) + " elements");
        }
        this.timestampNanos = timestampNanos;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.velocity = velocity;
        this.gyroBias = gyroBias;
        this.covariance = Arrays.copyOf(covariance, covariance.length);
        this.status = status;
        this.reliability = reliability;
    }

    public long timestampNanos() {
        return timestampNanos;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public double heading() {
        return heading;
    }

    public double velocity() {
        return velocity;
    }

    public double gyroBias() {
        return gyroBias;
    }

    /** Returns a copy of the row-major 5x5 covariance. */
    public double[] covariance() {
        return Arrays.copyOf(covariance, covariance.length);
    }

    public EstimatorStatus status() {
        return status;
    }

    public double reliability() {
        return reliability;
    }

    /** Standard deviation of the x position component. */
    public double positionStdX() {
        return Math.sqrt(covariance[0]);
    }

    /** Standard deviation of the y position component. */
    public double positionStdY() {
        return Math.sqrt(covariance[1 * DIM + 1]);
    }

    /** Standard deviation of the heading estimate (rad). */
    public double headingStd() {
        return Math.sqrt(covariance[2 * DIM + 2]);
    }

    /** Standard deviation of the gyro bias estimate (rad/s). */
    public double gyroBiasStd() {
        return Math.sqrt(covariance[4 * DIM + 4]);
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) {
            return true;
        }
        if (!(o instanceof PoseEstimate other)) {
            return false;
        }
        return timestampNanos == other.timestampNanos
                && Double.compare(x, other.x) == 0
                && Double.compare(y, other.y) == 0
                && Double.compare(heading, other.heading) == 0
                && Double.compare(velocity, other.velocity) == 0
                && Double.compare(gyroBias, other.gyroBias) == 0
                && Arrays.equals(covariance, other.covariance)
                && status == other.status
                && Double.compare(reliability, other.reliability) == 0;
    }

    @Override
    public int hashCode() {
        int result = Long.hashCode(timestampNanos);
        result = 31 * result + Double.hashCode(x);
        result = 31 * result + Double.hashCode(y);
        result = 31 * result + Double.hashCode(heading);
        result = 31 * result + Double.hashCode(velocity);
        result = 31 * result + Double.hashCode(gyroBias);
        result = 31 * result + Arrays.hashCode(covariance);
        result = 31 * result + status.hashCode();
        result = 31 * result + Double.hashCode(reliability);
        return result;
    }

    @Override
    public String toString() {
        return "PoseEstimate{t=" + timestampNanos
                + ", x=" + x
                + ", y=" + y
                + ", heading=" + heading
                + ", v=" + velocity
                + ", bg=" + gyroBias
                + ", status=" + status
                + ", reliability=" + reliability + '}';
    }
}
