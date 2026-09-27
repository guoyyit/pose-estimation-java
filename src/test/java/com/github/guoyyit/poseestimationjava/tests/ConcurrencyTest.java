package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.SensorType;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/** Concurrent inputs and queries: readers must never see half-updated state. */
class ConcurrencyTest {

    private static List<TestRig.Gen> scenario() {
        return TestRig.generate(61L, 1_000_000_000L, List.of(
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02),
                new TestRig.Segment(6.0, 0.5, 0.15, 0.02),
                new TestRig.Segment(2.0, 0.0, 0.0, 0.02)));
    }

    private static void submitOne(PoseEstimator est, TestRig.Gen g) {
        if (g.type() == SensorType.WHEEL) {
            est.submitWheelSpeed(new WheelSpeedSample(g.timestampNanos(), g.a(), g.b()));
        } else {
            est.submitImu(new ImuSample(g.timestampNanos(), g.a(), g.b(), g.c()));
        }
    }

    @Test
    void queriesConcurrentWithInputSeeConsistentSnapshots() throws Exception {
        List<TestRig.Gen> stream = scenario();
        PoseEstimator reference = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(reference, stream);
        reference.flush();
        PoseEstimate refFinal = reference.currentEstimate().orElseThrow();

        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        ExecutorService pool = Executors.newFixedThreadPool(3);
        CountDownLatch start = new CountDownLatch(1);
        AtomicBoolean failed = new AtomicBoolean(false);
        List<List<Long>> observedTimelines = new CopyOnWriteArrayList<>();

        // One writer thread feeds the stream...
        Future<?> writer = pool.submit(() -> {
            try {
                start.await();
                for (TestRig.Gen g : stream) {
                    submitOne(est, g);
                }
            } catch (Exception e) {
                failed.set(true);
            }
        });

        // ...while two readers query continuously.
        List<Future<?>> readers = new ArrayList<>();
        for (int r = 0; r < 2; r++) {
            readers.add(pool.submit(() -> {
                try {
                    start.await();
                    List<Long> timeline = new ArrayList<>();
                    for (int i = 0; i < 3000; i++) {
                        est.currentEstimate().ifPresent(s -> {
                            if (s.covariance().length != PoseEstimate.DIM * PoseEstimate.DIM
                                    || s.status() == null) {
                                failed.set(true); // half-updated snapshot
                            }
                            timeline.add(s.timestampNanos());
                        });
                    }
                    observedTimelines.add(timeline);
                } catch (Exception e) {
                    failed.set(true);
                }
            }));
        }

        start.countDown();
        writer.get(60, TimeUnit.SECONDS);
        for (Future<?> r : readers) {
            r.get(60, TimeUnit.SECONDS);
        }
        pool.shutdown();
        assertTrue(pool.awaitTermination(30, TimeUnit.SECONDS));
        assertFalse(failed.get(), "workers must not fail");

        for (List<Long> timeline : observedTimelines) {
            assertFalse(timeline.isEmpty());
            long prev = -1;
            for (long t : timeline) {
                assertTrue(t >= prev, "reader must observe a monotonic timeline");
                prev = t;
            }
        }

        est.flush();
        assertEquals(refFinal, est.currentEstimate().orElseThrow());
    }

    @Test
    void duplicateStreamsFromConcurrentWritersConvergeToSameState() throws Exception {
        List<TestRig.Gen> stream = scenario();
        PoseEstimator reference = new PoseEstimator(TestRig.testConfig());
        TestRig.feed(reference, stream);
        reference.flush();
        PoseEstimate refFinal = reference.currentEstimate().orElseThrow();

        // Two writers each feed the complete stream; every sample must be
        // processed exactly once, so the final state equals the reference.
        PoseEstimator est = new PoseEstimator(TestRig.testConfig());
        ExecutorService pool = Executors.newFixedThreadPool(2);
        CountDownLatch start = new CountDownLatch(1);
        List<Future<?>> futures = new ArrayList<>();
        for (int w = 0; w < 2; w++) {
            futures.add(pool.submit(() -> {
                try {
                    start.await();
                    for (TestRig.Gen g : stream) {
                        submitOne(est, g);
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }));
        }
        start.countDown();
        for (Future<?> f : futures) {
            f.get(60, TimeUnit.SECONDS);
        }
        pool.shutdown();
        assertTrue(pool.awaitTermination(30, TimeUnit.SECONDS));

        est.flush();
        assertEquals(refFinal, est.currentEstimate().orElseThrow());
    }
}
