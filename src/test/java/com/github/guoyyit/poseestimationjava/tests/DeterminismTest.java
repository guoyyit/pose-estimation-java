package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Fixed seed + identical input must produce identical output sequences. */
class DeterminismTest {

    @Test
    void sameSeedSameInputProducesIdenticalOutputSequence() {
        List<TestRig.Gen> stream1 = TestRig.generate(71L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(4.0, 0.5, 0.2, 0.03),
                new TestRig.Segment(2.0, 0.0, 0.0, 0.04)));
        // Regenerated with the same seed: the input itself must be identical.
        List<TestRig.Gen> stream2 = TestRig.generate(71L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(4.0, 0.5, 0.2, 0.03),
                new TestRig.Segment(2.0, 0.0, 0.0, 0.04)));
        assertEquals(stream1, stream2, "generator must be deterministic");

        PoseEstimator est1 = new PoseEstimator(TestRig.testConfig());
        PoseEstimator est2 = new PoseEstimator(TestRig.testConfig());
        List<PoseEstimate> out1 = TestRig.feedAndCollect(est1, stream1);
        List<PoseEstimate> out2 = TestRig.feedAndCollect(est2, stream2);

        assertTrue(out1.size() > 100);
        assertEquals(out1.size(), out2.size());
        for (int i = 0; i < out1.size(); i++) {
            assertEquals(out1.get(i), out2.get(i), "snapshot " + i + " must be identical");
        }
    }
}
