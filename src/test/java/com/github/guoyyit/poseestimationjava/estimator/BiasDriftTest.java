package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.EstimateStatus;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 零偏漂移：零偏未知时启动为未收敛，静止段在线估计并收敛，状态转为可用。
 */
class BiasDriftTest {

    @Test
    void unknownBiasStartsUnconvergedThenConverges() {
        PoseConfig cfg = PoseConfig.defaults()
                .withBiasInit(0.0, 1.0)              // 零偏未知
                .withHeadingInit(0.0, 1.0e-8);
        PoseEstimator est = new PoseEstimator(cfg);
        double trueBias = 0.005;
        long step = 20_000_000L;

        // 前 5 个采样（< 驻留时间）即便静止，状态仍应处于未收敛
        for (int i = 0; i < 5; i++) {
            est.ingest(new WheelSample(i * step, 0.0));
            est.ingest(new ImuSample(i * step, trueBias));
        }
        PoseEstimate early = est.query(4 * step, java.util.List.of());
        assertEquals(EstimateStatus.UNCONVERGED, early.status(),
                "零偏大方差时启动应为 UNCONVERGED");

        // 持续静止 3 秒 => 零偏收敛
        for (int i = 5; i < 150; i++) {
            est.ingest(new WheelSample(i * step, 0.0));
            est.ingest(new ImuSample(i * step, trueBias));
        }
        PoseEstimate late = est.query(149 * step, java.util.List.of());
        assertEquals(EstimateStatus.USABLE, late.status(),
                "静止 ZUPT 后零偏应收敛为 USABLE");
        assertEquals(trueBias, est.currentBiasMean(), 1.0e-3,
                "零偏均值应跟踪真实漂移");
        assertTrue(late.covarianceDiagonal()[4] <= cfg.biasConvergedVariance);
    }
}
