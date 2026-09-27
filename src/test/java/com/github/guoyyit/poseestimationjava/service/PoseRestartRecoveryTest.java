package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.io.TempDir;

import java.nio.file.Path;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;

/**
 * 重启恢复：从最近一次持久化检查点恢复，恢复后输出与重启前连续、无位姿跳变。
 */
class PoseRestartRecoveryTest {

    private void feedArc(PoseEstimationService svc, int n) {
        long step = 20_000_000L;
        for (int i = 0; i < n; i++) {
            svc.pushWheel(new WheelSample(i * step, 1.0));
            svc.pushImu(new ImuSample(i * step, 0.1)); // 缓慢转弯
        }
    }

    @Test
    void recoveryContinuesWithoutPoseJump(@TempDir Path dir) {
        String file = dir.resolve("pose-state.json").toString();
        PoseConfig cfg = PoseConfig.defaults();

        PoseEstimationService first = new PoseEstimationService(cfg, file);
        feedArc(first, 100); // 2 秒
        first.persist();
        PoseEstimate before = first.latest();
        assertNotNull(before);
        double xb = before.x();
        double yb = before.y();
        double tb = before.theta();
        long tbTime = before.timeNanos();

        // 模拟重启：用同一状态文件构造新服务
        PoseEstimationService second = new PoseEstimationService(cfg, file);
        PoseEstimate restored = second.queryAt(tbTime);
        assertNotNull(restored, "恢复后在重启前最后时刻必须有连续输出");

        // 恢复点与重启前逐分量一致（无跳变）
        assertEquals(xb, restored.x(), 1.0e-9, "x 恢复后必须连续");
        assertEquals(yb, restored.y(), 1.0e-9, "y 恢复后必须连续");
        assertEquals(tb, restored.theta(), 1.0e-9, "theta 恢复后必须连续");
        assertEquals(before.speed(), restored.speed(), 1.0e-9);

        // 恢复后继续喂入，轨迹沿原状态平滑延续
        long step = 20_000_000L;
        for (int i = 100; i < 120; i++) {
            second.pushWheel(new WheelSample(i * step, 1.0));
            second.pushImu(new ImuSample(i * step, 0.1));
        }
        PoseEstimate after = second.latest();
        // 继续转弯后航向单调增加、无跳变
        assertEquals(after.theta() > tb, true);
        // 时间轴连续推进
        assertEquals(119 * step, after.timeNanos());
    }

    @Test
    void noStateFileStartsFresh(@TempDir Path dir) {
        String file = dir.resolve("absent.json").toString();
        PoseEstimationService svc = new PoseEstimationService(PoseConfig.defaults(), file);
        // 无状态文件、无数据 => 查询返回 null（尚未建立时间轴）
        assertEquals(null, svc.latest());
    }
}
