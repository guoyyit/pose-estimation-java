package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.EstimatorStatus;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Initial heading/bias estimation, convergence and covariance reporting. */
class BiasConvergenceTest {

    @Test
    void stationaryStartEstimatesBiasAndConverges() {
        // True gyro bias 0.05 rad/s, filter prior 0 with large variance.
        List<TestRig.Gen> stream = TestRig.generate(11L, 1_000_000_000L, List.of(
                new TestRig.Segment(6.0, 0.0, 0.0, 0.05)));
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        List<PoseEstimate> snapshots = TestRig.feedAndCollect(est, stream);

        // Early snapshots must report UNCONVERGED, later ones TRACKING.
        assertEquals(EstimatorStatus.UNCONVERGED, snapshots.get(0).status());
        PoseEstimate last = snapshots.get(snapshots.size() - 1);
        assertEquals(EstimatorStatus.TRACKING, last.status());

        // Bias estimate converges to the true value.
        assertEquals(0.05, last.gyroBias(), 0.01);

        // Covariance shrinks monotonically enough to be visible in the output.
        PoseEstimate mid = snapshots.get(snapshots.size() / 2);
        assertTrue(last.gyroBiasStd() < mid.gyroBiasStd(),
                "bias std should shrink: " + mid.gyroBiasStd() + " -> " + last.gyroBiasStd());
        assertTrue(last.headingStd() < Math.sqrt(0.02),
                "heading std below convergence threshold: " + last.headingStd());
        assertTrue(last.reliability() > 0.0 && last.reliability() <= 1.0);
    }

    @Test
    void biasDriftIsTracked() {
        // Converge while stationary, then bias ramps 0.02 -> 0.08 while moving.
        List<TestRig.Segment> segments = new ArrayList<>();
        segments.add(new TestRig.Segment(3.0, 0.0, 0.0, 0.02));
        int steps = 20;
        for (int i = 0; i < steps; i++) {
            double bias = 0.02 + (0.08 - 0.02) * i / (steps - 1.0);
            segments.add(new TestRig.Segment(0.5, 0.4, 0.05, bias));
        }
        List<TestRig.Gen> stream = TestRig.generate(13L, 1_000_000_000L, segments);
        PoseEstimatorConfig cfg = TestRig.testConfig();
        cfg.gyroBiasRandomWalkVar = 1e-4; // let the filter track the drift
        PoseEstimator est = new PoseEstimator(cfg);
        TestRig.feed(est, stream);
        est.flush();

        PoseEstimate last = est.currentEstimate().orElseThrow();
        assertEquals(0.08, last.gyroBias(), 0.02,
                "bias estimate should track the drift");
        assertEquals(EstimatorStatus.TRACKING, last.status());
    }

    @Test
    void covarianceStaysPositiveAndFinite() {
        List<TestRig.Gen> stream = TestRig.generate(17L, 1_000_000_000L, List.of(
                new TestRig.Segment(3.0, 0.0, 0.0, 0.03),
                new TestRig.Segment(3.0, 0.6, 0.2, 0.03)));
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        List<PoseEstimate> snapshots = TestRig.feedAndCollect(est, stream);
        for (PoseEstimate s : snapshots) {
            double[] cov = s.covariance();
            for (int i = 0; i < PoseEstimate.DIM; i++) {
                double d = cov[i * PoseEstimate.DIM + i];
                assertTrue(d > 0.0 && Double.isFinite(d), "variance must stay positive/finite");
            }
        }
    }
}
