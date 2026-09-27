package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import com.github.guoyyit.poseestimationjava.simulation.TrajectorySimulator;
import org.junit.jupiter.api.Test;

import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;

/**
 * 确定性：相同随机种子与相同输入，两次运行输出逐条一致（位级别）。
 */
class DeterminismTest {

    @Test
    void sameSeedSameInputsProduceBitIdenticalOutputs() {
        PoseConfig cfg = PoseConfig.defaults().withRandomSeed(12345L);
        long duration = 3_000_000_000L;

        PoseEstimator a = run(cfg, duration);
        PoseEstimator b = run(cfg, duration);

        long end = a.lastInputTime();
        for (long q = 0; q <= end; q += 50_000_000L) {
            PoseEstimate ea = a.query(q, List.of());
            PoseEstimate eb = b.query(q, List.of());
            assertEquals(ea.timeNanos(), eb.timeNanos());
            // 完全相同输入 => 位级别一致
            assertEquals(ea.x(), eb.x(), 0.0);
            assertEquals(ea.y(), eb.y(), 0.0);
            assertEquals(ea.theta(), eb.theta(), 0.0);
            assertEquals(ea.speed(), eb.speed(), 0.0);
        }
    }

    private PoseEstimator run(PoseConfig cfg, long duration) {
        TrajectorySimulator sim = new TrajectorySimulator(cfg);
        List<WheelSample> wheels = sim.arcWheels(0, duration, 1.0);
        List<ImuSample> imus = sim.arcImus(0, duration, 0.1, 0.001);
        PoseEstimator e = new PoseEstimator(cfg);
        for (WheelSample w : wheels) {
            e.ingest(w);
        }
        for (ImuSample i : imus) {
            e.ingest(i);
        }
        return e;
    }
}
