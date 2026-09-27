package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyReason;
import com.github.guoyyit.poseestimationjava.model.EstimatorStatus;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.SensorType;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Sensor anomaly detection: bad samples must not pollute the pose. */
class SensorAnomalyTest {

    @Test
    void wheelSpeedJumpIsRejected() {
        List<TestRig.Gen> clean = TestRig.generate(21L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(3.0, 0.4, 0.0, 0.02)));

        PoseEstimator baseline = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(baseline, clean);
        baseline.flush();
        PoseEstimate baseFinal = baseline.currentEstimate().orElseThrow();

        // Corrupt one wheel sample in the moving phase with a 5 m/s spike.
        List<TestRig.Gen> corrupted = new ArrayList<>(clean);
        int idx = -1;
        for (int i = 0; i < corrupted.size(); i++) {
            TestRig.Gen g = corrupted.get(i);
            if (g.type() == SensorType.WHEEL && g.timestampNanos() > 3_000_000_000L) {
                idx = i;
                break;
            }
        }
        assertTrue(idx > 0);
        TestRig.Gen g = corrupted.get(idx);
        corrupted.set(idx, new TestRig.Gen(g.timestampNanos(), SensorType.WHEEL, 5.0, 5.0, 0.0));

        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(est, corrupted);
        est.flush();

        List<AnomalyEvent> anomalies = est.drainAnomalies();
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.WHEEL_SPEED_JUMP),
                "expected WHEEL_SPEED_JUMP, got: " + anomalies);

        PoseEstimate finalEst = est.currentEstimate().orElseThrow();
        assertEquals(baseFinal.x(), finalEst.x(), 0.05, "pose must not be polluted");
        assertEquals(baseFinal.y(), finalEst.y(), 0.05);
        assertEquals(baseFinal.heading(), finalEst.heading(), 0.02);
    }

    @Test
    void gyroBiasJumpIsDetectedAndPoseStaysConsistent() {
        List<TestRig.Gen> clean = TestRig.generate(23L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(3.0, 0.4, 0.1, 0.02)));
        // Same stream but the gyro bias jumps by +1.0 rad/s in the moving phase.
        List<TestRig.Gen> jumped = TestRig.generate(23L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(3.0, 0.4, 0.1, 1.02)));

        PoseEstimator baseline = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(baseline, clean);
        baseline.flush();
        PoseEstimate baseFinal = baseline.currentEstimate().orElseThrow();

        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(est, jumped);
        est.flush();

        List<AnomalyEvent> anomalies = est.drainAnomalies();
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.GYRO_BIAS_JUMP),
                "expected GYRO_BIAS_JUMP, got: " + anomalies);

        // The bias is re-leveled, so the estimate follows the jumped gyro...
        PoseEstimate finalEst = est.currentEstimate().orElseThrow();
        assertEquals(1.02, finalEst.gyroBias(), 0.1, "bias should be re-leveled");
        // ...while the pose itself stays consistent with the unpolluted run.
        assertEquals(baseFinal.heading(), finalEst.heading(), 0.1, "heading must not jump");
        assertEquals(baseFinal.x(), finalEst.x(), 0.2);
        assertEquals(baseFinal.y(), finalEst.y(), 0.2);
    }

    @Test
    void longNoInputIsReportedAndRecovers() {
        List<TestRig.Gen> stream = TestRig.generate(29L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02)));
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(est, stream);
        est.flush();
        assertEquals(EstimatorStatus.TRACKING, est.status());
        est.drainAnomalies();

        long lastTs = stream.get(stream.size() - 1).timestampNanos();
        // Advance the clock beyond the 1 s no-input timeout.
        est.onTimer(lastTs + 1_500_000_000L);
        assertEquals(EstimatorStatus.DEGRADED, est.status());
        List<AnomalyEvent> anomalies = est.drainAnomalies();
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.NO_INPUT),
                "expected NO_INPUT, got: " + anomalies);

        // Input resumes: status recovers and the episode does not repeat.
        List<TestRig.Gen> more = TestRig.generate(31L, lastTs + 1_600_000_000L, List.of(
                new TestRig.Segment(1.0, 0.0, 0.0, 0.02)));
        TestRig.feed(est, more);
        est.flush();
        assertEquals(EstimatorStatus.TRACKING, est.status());
    }
}
