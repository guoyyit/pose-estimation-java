package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyReason;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Out-of-order, duplicate and timestamp-rollback inputs must not break continuity. */
class OutOfOrderTest {

    private static List<TestRig.Gen> scenario() {
        return TestRig.generate(42L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(3.0, 0.5, 0.1, 0.02)));
    }

    @Test
    void reorderWithinWindowYieldsIdenticalOutput() {
        List<TestRig.Gen> ordered = scenario();

        PoseEstimator baseline = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(baseline, ordered);
        baseline.flush();

        // Shuffle inside chunks shorter than the reorder window (100 ms).
        List<TestRig.Gen> shuffled = new ArrayList<>();
        Random rnd = new Random(7L);
        int chunk = 5; // 5 samples * 10 ms = 50 ms < 100 ms window
        for (int i = 0; i < ordered.size(); i += chunk) {
            List<TestRig.Gen> part = new ArrayList<>(ordered.subList(i, Math.min(i + chunk, ordered.size())));
            Collections.shuffle(part, rnd);
            shuffled.addAll(part);
        }

        PoseEstimator reordered = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(reordered, shuffled);
        reordered.flush();

        assertEquals(baseline.currentEstimate(), reordered.currentEstimate());
    }

    @Test
    void duplicatesAreRejectedAndDoNotChangeOutput() {
        List<TestRig.Gen> ordered = scenario();

        PoseEstimator baseline = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(baseline, ordered);
        baseline.flush();

        PoseEstimator duped = new PoseEstimator(TestRig.testConfig());
        for (TestRig.Gen g : ordered) {
            TestRig.feed(duped, List.of(g, g, g)); // each sample three times
        }
        duped.flush();

        List<AnomalyEvent> anomalies = duped.drainAnomalies();
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.DUPLICATE),
                "expected DUPLICATE anomalies, got: " + anomalies);
        assertEquals(baseline.currentEstimate(), duped.currentEstimate());
    }

    @Test
    void timestampRollbackIsRejectedAndEstimateUnchanged() {
        List<TestRig.Gen> ordered = scenario();
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(est, ordered);
        est.flush();
        PoseEstimate before = est.currentEstimate().orElseThrow();
        est.drainAnomalies();

        // Severe rollback: far older than committed time minus reorder window.
        long rolledBack = ordered.get(ordered.size() / 2).timestampNanos();
        est.submitImu(new ImuSample(rolledBack, 0.0, 0.0, 0.0));
        // Mild lateness: just behind the committed time.
        est.submitWheelSpeed(new WheelSpeedSample(before.timestampNanos(), 0.1, 0.1));

        List<AnomalyEvent> anomalies = est.drainAnomalies();
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.TIMESTAMP_ROLLBACK),
                "expected TIMESTAMP_ROLLBACK, got: " + anomalies);
        assertTrue(anomalies.stream().anyMatch(a -> a.reason() == AnomalyReason.OUT_OF_ORDER),
                "expected OUT_OF_ORDER, got: " + anomalies);
        assertEquals(before, est.currentEstimate().orElseThrow());
    }

    @Test
    void committedTimelineIsMonotonic() {
        List<TestRig.Gen> ordered = scenario();
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        List<PoseEstimate> snapshots = TestRig.feedAndCollect(est, ordered);
        assertTrue(snapshots.size() > 10);
        long prev = -1;
        for (PoseEstimate s : snapshots) {
            assertTrue(s.timestampNanos() >= prev, "timeline must be monotonic");
            prev = s.timestampNanos();
        }
    }
}
