package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

/**
 * 配置热更新：变更只作用于后续估计，已提交状态不发生跳变。
 */
class ConfigUpdateTest {

    @Test
    void configChangeAppliesOnlyAfterwards() {
        PoseEstimationService svc = new PoseEstimationService(PoseConfig.defaults(), null);
        long step = 20_000_000L;
        for (int i = 0; i < 50; i++) {
            svc.pushWheel(new WheelSample(i * step, 1.0));
            svc.pushImu(new ImuSample(i * step, 0.0));
        }
        PoseEstimate before = svc.latest();

        // 热更新：更严的轮速突变阈值
        PoseConfig updated = svc.currentConfig()
                .withThresholds(0.5, 0.02, 0.02);
        svc.updateConfig(updated);
        assertEquals(0.5, svc.currentConfig().wheelJumpThreshold, 0.0);

        // 已提交状态不跳变
        PoseEstimate sameTime = svc.queryAt(before.timeNanos());
        assertEquals(before.x(), sameTime.x(), 1.0e-9);
        assertEquals(before.y(), sameTime.y(), 1.0e-9);
        assertEquals(before.theta(), sameTime.theta(), 1.0e-9);

        // 新阈值生效：0->2 的变化（>0.5）被判突变
        var jump = svc.pushWheel(new WheelSample(1_000_000_000L, 2.0));
        assertNotNull(jump.orElse(null));
        assertEquals(com.github.guoyyit.poseestimationjava.model.AnomalyType.WHEEL_SPEED_JUMP,
                jump.get().type());
    }
}
