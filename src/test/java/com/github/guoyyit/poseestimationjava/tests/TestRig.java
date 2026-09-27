package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.SensorType;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * Deterministic input-stream generator shared by the estimator tests.
 * All randomness flows from an explicit seed so runs are reproducible.
 */
final class TestRig {

    private TestRig() {
    }

    /** A generated sample, unified across sensor types. */
    record Gen(long timestampNanos, SensorType type, double a, double b, double c) {
    }

    /** Motion profile segment: constant forward speed and yaw rate. */
    record Segment(double seconds, double speed, double yawRate, double gyroBias) {
    }

    static PoseEstimatorConfig testConfig() {
        PoseEstimatorConfig c = PoseEstimatorConfig.defaults();
        c.reorderWindowNanos = 100_000_000L;      // 100 ms
        c.stationaryWindowNanos = 300_000_000L;   // 300 ms
        c.noInputTimeoutNanos = 1_000_000_000L;   // 1 s
        c.initialHeadingVariance = 0.5;
        c.initialGyroBiasVariance = 0.2;
        c.headingConvergedVariance = 0.02;
        c.biasConvergedVariance = 0.005;
        return c;
    }

    /**
     * Generates an interleaved wheel (50 Hz) + IMU (100 Hz) stream.
     *
     * @param seed     fixed random seed
     * @param startNanos first timestamp
     * @param segments motion profile
     * @return samples in strictly increasing timestamp order
     */
    static List<Gen> generate(long seed, long startNanos, List<Segment> segments) {
        return generate(seed, startNanos, PoseEstimatorConfig.defaults(), segments);
    }

    static List<Gen> generate(long seed, long startNanos, PoseEstimatorConfig cfg,
                              List<Segment> segments) {
        Random rnd = new Random(seed);
        List<Gen> out = new ArrayList<>();
        long wheelPeriod = 20_000_000L;  // 50 Hz
        long imuPeriod = 10_000_000L;    // 100 Hz
        long t = startNanos;
        long nextWheel = t;
        long nextImu = t;
        for (Segment seg : segments) {
            long end = t + (long) (seg.seconds() * 1e9);
            while (Math.min(nextWheel, nextImu) < end) {
                if (nextImu <= nextWheel) {
                    double gyro = seg.yawRate() + seg.gyroBias()
                            + rnd.nextGaussian() * Math.sqrt(cfg.gyroNoiseVar);
                    out.add(new Gen(nextImu, SensorType.IMU, gyro,
                            rnd.nextGaussian() * 0.01, rnd.nextGaussian() * 0.01));
                    nextImu += imuPeriod;
                } else {
                    double half = seg.yawRate() * cfg.trackWidth / 2.0;
                    double left = seg.speed() - half
                            + rnd.nextGaussian() * Math.sqrt(cfg.wheelSpeedNoiseVar);
                    double right = seg.speed() + half
                            + rnd.nextGaussian() * Math.sqrt(cfg.wheelSpeedNoiseVar);
                    out.add(new Gen(nextWheel, SensorType.WHEEL, left, right, 0.0));
                    nextWheel += wheelPeriod;
                }
            }
            t = end;
        }
        return out;
    }

    /** Feeds samples into the service in the given order. */
    static void feed(com.github.guoyyit.poseestimationjava.service.PoseEstimatorService service,
                     List<Gen> samples) {
        for (Gen g : samples) {
            if (g.type() == SensorType.WHEEL) {
                service.submitWheelSpeed(new WheelSpeedSample(g.timestampNanos(), g.a(), g.b()));
            } else {
                service.submitImu(new ImuSample(g.timestampNanos(), g.a(), g.b(), g.c()));
            }
        }
    }

    /** Feeds samples into the estimator in the given order. */
    static void feed(PoseEstimator estimator, List<Gen> samples) {
        for (Gen g : samples) {
            if (g.type() == SensorType.WHEEL) {
                estimator.submitWheelSpeed(new WheelSpeedSample(g.timestampNanos(), g.a(), g.b()));
            } else {
                estimator.submitImu(new ImuSample(g.timestampNanos(), g.a(), g.b(), g.c()));
            }
        }
    }

    /** Collects one snapshot per committed sample by flushing progressively. */
    static List<com.github.guoyyit.poseestimationjava.model.PoseEstimate> feedAndCollect(
            PoseEstimator estimator, List<Gen> samples) {
        List<com.github.guoyyit.poseestimationjava.model.PoseEstimate> out = new ArrayList<>();
        for (Gen g : samples) {
            if (g.type() == SensorType.WHEEL) {
                estimator.submitWheelSpeed(new WheelSpeedSample(g.timestampNanos(), g.a(), g.b()));
            } else {
                estimator.submitImu(new ImuSample(g.timestampNanos(), g.a(), g.b(), g.c()));
            }
            estimator.currentEstimate().ifPresent(out::add);
        }
        estimator.flush();
        estimator.currentEstimate().ifPresent(out::add);
        return out;
    }
}
