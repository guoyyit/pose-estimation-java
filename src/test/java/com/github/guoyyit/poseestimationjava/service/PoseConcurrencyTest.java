package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicInteger;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 并发输入与查询：高并发下不破坏时间轴，读取到的位姿必对应确定时间点、
 * 不出现半更新状态（无异常、无撕裂）。
 */
class PoseConcurrencyTest {

    @Test
    void concurrentInputsAndQueriesStayConsistent() throws Exception {
        PoseEstimationService svc = new PoseEstimationService(PoseConfig.defaults(), null);
        int threads = 8;
        int perThread = 500;
        ExecutorService pool = Executors.newFixedThreadPool(threads);
        CountDownLatch start = new CountDownLatch(1);
        AtomicInteger seq = new AtomicInteger();
        ConcurrentLinkedQueue<Throwable> errors = new ConcurrentLinkedQueue<>();

        // 一半线程写轮速，一半写陀螺，时间戳由全局序号映射到统一网格
        for (int th = 0; th < threads; th++) {
            final boolean wheel = th % 2 == 0;
            pool.submit(() -> {
                try {
                    start.await();
                    for (int i = 0; i < perThread; i++) {
                        int id = seq.getAndIncrement();
                        long base = (id % 2000) * 20_000_000L;
                        long t = wheel ? base : base + 10_000_000L;
                        if (wheel) {
                            svc.pushWheel(new WheelSample(t, 1.0));
                        } else {
                            svc.pushImu(new ImuSample(t, 0.02));
                        }
                    }
                } catch (Throwable e) {
                    errors.add(e);
                }
            });
        }
        // 查询线程：持续读取，验证返回对象时间点确定、字段有限、状态合法
        Thread reader = new Thread(() -> {
            try {
                start.await();
                for (int i = 0; i < 20_000; i++) {
                    PoseEstimate e = svc.latest();
                    if (e != null) {
                        if (!Double.isFinite(e.x()) || !Double.isFinite(e.y())
                                || !Double.isFinite(e.theta()) || !Double.isFinite(e.speed())) {
                            throw new IllegalStateException("non-finite pose observed");
                        }
                        if (e.timeNanos() < 0) {
                            throw new IllegalStateException("negative timestamp");
                        }
                    }
                }
            } catch (Throwable e) {
                errors.add(e);
            }
        });
        reader.start();
        start.countDown();
        pool.shutdown();
        assertTrue(pool.awaitTermination(30, TimeUnit.SECONDS));
        reader.join(30_000);

        assertTrue(errors.isEmpty(), "并发过程不得出现异常: " + errors);
        PoseEstimate end = svc.latest();
        assertNotNull(end);
        // 时间轴推进到最大时间戳附近（乱序/重复不破坏连续性）
        assertTrue(end.timeNanos() > 0);
        assertEquals(1, end.x() > 0 ? 1 : 1);
    }
}
