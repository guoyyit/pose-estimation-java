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

/** A stationary platform must not drift; covariance must stay bounded. */
class StationaryTest {

    @Test
    void stationaryPlatformDoesNotDrift() {
        // 30 s perfectly still with a constant gyro bias of 0.03 rad/s.
        List<TestRig.Gen> stream = TestRig.generate(37L, 1_000_000_000L, List.of(
                new TestRig.Segment(30.0, 0.0, 0.0, 0.03)));
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        List<PoseEstimate> snapshots = TestRig.feedAndCollect(est, stream);

        PoseEstimate last = snapshots.get(snapshots.size() - 1);
        assertEquals(EstimatorStatus.TRACKING, last.status());
        assertEquals(0.0, last.x(), 0.05, "x must not drift");
        assertEquals(0.0, last.y(), 0.05, "y must not drift");
        assertEquals(0.0, last.velocity(), 0.01, "velocity must stay zero");
        assertEquals(0.03, last.gyroBias(), 0.005, "bias must be estimated");

        // After convergence (t=10s) there must be no ongoing drift at all:
        // compare the estimate 20 s later against the one at t=10s.
        PoseEstimate at10s = null;
        for (PoseEstimate snap : snapshots) {
            if (snap.timestampNanos() >= 11_000_000_000L) {
                at10s = snap;
                break;
            }
        }
        assertTrue(at10s != null);
        assertEquals(at10s.heading(), last.heading(), 0.005, "heading must not keep drifting");
        assertEquals(at10s.x(), last.x(), 0.01, "x must not keep drifting");
        assertEquals(at10s.y(), last.y(), 0.01, "y must not keep drifting");
    }

    @Test
    void covarianceStaysBoundedOverLongRun() {
        // 60 s of alternating motion and rest.
        List<TestRig.Segment> segments = new ArrayList<>();
        for (int i = 0; i < 6; i++) {
            segments.add(new TestRig.Segment(5.0, 0.5, 0.2, 0.03));
            segments.add(new TestRig.Segment(5.0, 0.0, 0.0, 0.03));
        }
        List<TestRig.Gen> stream = TestRig.generate(41L, 1_000_000_000L, segments);
        PoseEstimatorConfig cfg = TestRig.testConfig();
        PoseEstimator est = new PoseEstimator(cfg);
        List<PoseEstimate> snapshots = TestRig.feedAndCollect(est, stream);
        assertTrue(snapshots.size() > 1000);

        double maxHeadingVar = 0.0;
        for (PoseEstimate s : snapshots) {
            double[] cov = s.covariance();
            for (int i = 0; i < PoseEstimate.DIM; i++) {
                double d = cov[i * PoseEstimate.DIM + i];
                assertTrue(Double.isFinite(d) && d > 0.0, "variance must stay finite/positive");
            }
            assertTrue(cov[0] <= cfg.maxPositionVariance + 1e-9);
            assertTrue(cov[2 * PoseEstimate.DIM + 2] <= cfg.maxHeadingVariance + 1e-9,
                    "heading variance must not diverge");
            maxHeadingVar = Math.max(maxHeadingVar, cov[2 * PoseEstimate.DIM + 2]);
        }
        // Sanity: the run actually exercised covariance growth and recovery.
        assertTrue(maxHeadingVar > cfg.headingConvergedVariance);
        PoseEstimate last = snapshots.get(snapshots.size() - 1);
        assertEquals(EstimatorStatus.TRACKING, last.status());
    }
}
