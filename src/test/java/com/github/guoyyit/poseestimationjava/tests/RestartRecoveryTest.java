package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.model.AnomalyReason;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.SensorType;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;
import com.github.guoyyit.poseestimationjava.service.PoseEstimatorService;
import com.github.guoyyit.poseestimationjava.store.FileStateStore;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Restart recovery: resume from the persisted state without a pose jump. */
class RestartRecoveryTest {

    @TempDir
    Path dir;

    @Test
    void restartResumesWithoutJumpAndStaysContinuous() {
        Path stateFile = dir.resolve("pose-state.txt");
        List<TestRig.Gen> part1 = TestRig.generate(51L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(2.0, 0.4, 0.1, 0.02)));
        List<TestRig.Gen> part2 = TestRig.generate(53L, 5_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.4, -0.1, 0.02)));

        // Uninterrupted reference run.
        PoseEstimatorService reference = new PoseEstimatorService(
                TestRig.testConfig(), new FileStateStore(dir.resolve("ref.txt")));
        TestRig.feed(reference, part1);
        reference.flush();
        TestRig.feed(reference, part2);
        reference.flush();
        PoseEstimate refFinal = reference.currentEstimate().orElseThrow();

        // Run 1: feed part1, flush, then "shut down".
        PoseEstimatorService first = new PoseEstimatorService(
                TestRig.testConfig(), new FileStateStore(stateFile));
        TestRig.feed(first, part1);
        first.flush();
        PoseEstimate beforeRestart = first.currentEstimate().orElseThrow();

        // Run 2: a fresh service over the same state file simulates the restart.
        PoseEstimatorService second = new PoseEstimatorService(
                TestRig.testConfig(), new FileStateStore(stateFile));
        PoseEstimate afterRestart = second.currentEstimate().orElseThrow();

        // No pose jump: the restored estimate equals the pre-restart one exactly.
        assertEquals(beforeRestart, afterRestart);

        // Continuity: feeding part2 after the restart matches the reference run.
        TestRig.feed(second, part2);
        second.flush();
        assertEquals(refFinal, second.currentEstimate().orElseThrow());
    }

    @Test
    void configChangeAppliesToSubsequentEstimates() {
        PoseEstimatorService service = new PoseEstimatorService(
                TestRig.testConfig(), new FileStateStore(dir.resolve("cfg.txt")));
        List<TestRig.Gen> stream = TestRig.generate(57L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02)));
        TestRig.feed(service, stream);
        service.flush();
        service.drainAnomalies();

        // A 5 m/s spike exceeds the default 1 m/s wheel jump gate.
        long t = 3_000_000_000L;
        service.submitWheelSpeed(new WheelSpeedSample(t, 5.0, 5.0));
        service.flush();
        assertTrue(service.drainAnomalies().stream()
                .anyMatch(a -> a.reason() == AnomalyReason.WHEEL_SPEED_JUMP));

        // Raise the gate: the same spike must now be accepted (no anomaly).
        PoseEstimatorConfig cfg = TestRig.testConfig();
        cfg.wheelJumpGate = 10.0;
        cfg.innovationGateSigma = 1e9; // disable the n-sigma part for the test
        service.updateConfig(cfg);
        service.submitWheelSpeed(new WheelSpeedSample(t + 200_000_000L, 5.0, 5.0));
        service.flush();
        assertTrue(service.drainAnomalies().stream()
                .noneMatch(a -> a.reason() == AnomalyReason.WHEEL_SPEED_JUMP),
                "new config must apply to subsequent estimates");
    }
}
