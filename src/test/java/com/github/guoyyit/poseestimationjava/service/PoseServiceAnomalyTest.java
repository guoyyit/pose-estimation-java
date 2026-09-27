package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.AnomalyType;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Path;
import java.util.List;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class PoseServiceAnomalyTest {

    private PoseEstimationService service() {
        return new PoseEstimationService(PoseConfig.defaults(), null);
    }

    private void feedStationary(PoseEstimationService svc, int n) {
        long step = 20_000_000L;
        for (int i = 0; i < n; i++) {
            long t = i * step;
            svc.pushWheel(new WheelSample(t, 0.0));
            svc.pushImu(new ImuSample(t, 0.0));
        }
    }

    @Test
    void wheelSpeedJumpIsRejectedAndDoesNotPollutePose() {
        PoseEstimationService svc = service();
        feedStationary(svc, 5);
        PoseEstimate before = svc.latest();
        // 物理上不可能的突变（注入到最新时间之后）
        var rejected = svc.pushWheel(new WheelSample(100_000_000L, 50.0));
        assertTrue(rejected.isPresent());
        assertEquals(AnomalyType.WHEEL_SPEED_JUMP, rejected.get().type());
        // 继续正常输入
        svc.pushImu(new ImuSample(200_000_000L, 0.0));
        PoseEstimate after = svc.latest();
        // 速度未被突变污染（ZUPT 后仍约 0）
        assertEquals(0.0, after.speed(), 0.05, "突变轮速不得污染速度");
        assertEquals(before.x(), after.x(), 1.0e-6);
        assertTrue(svc.anomalies().stream()
                .anyMatch(e -> e.type() == AnomalyType.WHEEL_SPEED_JUMP));
    }

    @Test
    void gyroBiasJumpWhileStationaryIsRejected() {
        PoseEstimationService svc = service();
        feedStationary(svc, 15);
        // 静止段陀螺突然读到大值（远超零偏跳变阈值），注入到最新时间之后
        var rejected = svc.pushImu(new ImuSample(300_000_000L, 5.0));
        assertTrue(rejected.isPresent());
        assertEquals(AnomalyType.GYRO_BIAS_JUMP, rejected.get().type());
        PoseEstimate e = svc.latest();
        // 航向/位置不被污染
        assertEquals(0.0, e.theta(), 1.0e-3, "零偏跳变不得污染航向");
    }

    @Test
    void noInputTriggersTimeoutOnlyOnExplicitFutureQuery() {
        PoseEstimationService svc = service();
        feedStationary(svc, 10);
        long last = svc.latest().timeNanos();
        // 未超时：无 TIMEOUT
        PoseEstimate ok = svc.queryAt(last + 100_000_000L);
        assertNotNull(ok);
        assertTrue(ok.activeAnomalies().stream()
                .noneMatch(a -> a.type() == AnomalyType.INPUT_TIMEOUT));
        // 超过 1s 无输入：超时事件，原因可区分
        PoseEstimate late = svc.queryAt(last + 1_500_000_000L);
        assertTrue(late.activeAnomalies().stream()
                .anyMatch(a -> a.type() == AnomalyType.INPUT_TIMEOUT),
                "应报告 INPUT_TIMEOUT");
    }

    @Test
    void duplicateAndRollbackReasonsDistinguished() {
        PoseEstimationService svc = service();
        feedStationary(svc, 5);
        var dup = svc.pushImu(new ImuSample(40_000_000L, 0.0));
        assertEquals(AnomalyType.DUPLICATE_TIMESTAMP, dup.get().type());
        // 窗口内迟到（回拨），接纳并报告
        var rb = svc.pushWheel(new WheelSample(10_000_000L, 0.0));
        assertEquals(AnomalyType.TIMESTAMP_ROLLBACK, rb.get().type());
    }
}
