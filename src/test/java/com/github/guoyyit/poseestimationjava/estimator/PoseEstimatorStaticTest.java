package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.estimator.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.EstimateStatus;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 静止场景：零偏在线收敛、静止不漂移、协方差不发散。
 */
class PoseEstimatorStaticTest {

    private PoseConfig unknownBiasConfig() {
        return PoseConfig.defaults()
                // 零偏未知：初值 0，方差很大 => 启动 UNCONVERGED
                .withBiasInit(0.0, 1.0)
                // 真实零偏 0.01 rad/s 将通过静止 ZUPT 被估计出来
                .withHeadingInit(0.0, 1.0e-8);
    }

    @Test
    void gyroBiasConvergesDuringStationaryPeriod() {
        PoseConfig cfg = unknownBiasConfig();
        PoseEstimator est = new PoseEstimator(cfg);

        double trueBias = 0.01;
        long t = 0L;
        // 2 秒静止，陀螺读数 = 真实零偏（无噪声，确定性验证）
        for (int i = 0; i < 100; i++) {
            est.ingest(new WheelSample(t, 0.0));
            est.ingest(new ImuSample(t, trueBias));
            t += 20_000_000L;
        }

        var estimate = est.query(t - 20_000_000L, java.util.List.of());
        assertEquals(EstimateStatus.USABLE, estimate.status(),
                "零偏收敛后状态应为 USABLE");
        // 零偏方差收敛到阈值以下
        assertTrue(estimate.covarianceDiagonal()[4] < cfg.biasConvergedVariance,
                "零偏方差应收敛到阈值以下");
        // 零偏均值接近真实零偏
        assertEquals(trueBias, est.currentBiasMean(), 1.0e-3,
                "静止段应估计出真实陀螺零偏");
    }

    @Test
    void stationaryStateDoesNotDrift() {
        PoseConfig cfg = PoseConfig.defaults();
        PoseEstimator est = new PoseEstimator(cfg);
        long t = 0L;
        for (int i = 0; i < 200; i++) {
            est.ingest(new WheelSample(t, 0.0));
            est.ingest(new ImuSample(t, 0.0));
            t += 20_000_000L;
        }
        var e = est.query(t - 20_000_000L, java.util.List.of());
        assertEquals(0.0, e.x(), 1.0e-6, "静止 x 不得漂移");
        assertEquals(0.0, e.y(), 1.0e-6, "静止 y 不得漂移");
        assertEquals(0.0, e.theta(), 1.0e-6, "静止航向不得漂移");
        assertEquals(0.0, e.speed(), 1.0e-3, "静止速度应保持 0");
    }

    @Test
    void covarianceStaysBoundedLongRun() {
        PoseConfig cfg = PoseConfig.defaults();
        PoseEstimator est = new PoseEstimator(cfg);
        long t = 0L;
        // 30 秒运动+静止交替
        for (int i = 0; i < 1500; i++) {
            double v = (i / 100) % 2 == 0 ? 1.0 : 0.0;
            est.ingest(new WheelSample(t, v));
            est.ingest(new ImuSample(t, 0.0));
            t += 20_000_000L;
        }
        var e = est.query(t - 20_000_000L, java.util.List.of());
        for (double d : e.covarianceDiagonal()) {
            assertTrue(d <= cfg.covarianceCap * 1.0001, "协方差不得超过饱和上限: " + d);
            assertTrue(d >= cfg.covarianceFloor * 0.9999, "协方差不得低于下限: " + d);
        }
    }
}
