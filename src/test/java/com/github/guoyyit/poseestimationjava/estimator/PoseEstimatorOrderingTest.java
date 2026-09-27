package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

/**
 * 乱序/异步到达不破坏连续性，且结果与按序到达逐条一致（确定性）。
 */
class PoseEstimatorOrderingTest {

    private PoseEstimator newEstimator() {
        return new PoseEstimator(PoseConfig.defaults());
    }

    private List<WheelSample> wheels(List<Long> times, double speed) {
        List<WheelSample> l = new ArrayList<>();
        for (long t : times) {
            l.add(new WheelSample(t, speed));
        }
        return l;
    }

    private List<ImuSample> imus(List<Long> times, double gyro) {
        List<ImuSample> l = new ArrayList<>();
        for (long t : times) {
            l.add(new ImuSample(t, gyro));
        }
        return l;
    }

    private void feedOrdered(PoseEstimator e, List<WheelSample> ws, List<ImuSample> is) {
        int wi = 0, ii = 0;
        while (wi < ws.size() || ii < is.size()) {
            long tw = wi < ws.size() ? ws.get(wi).timeNanos() : Long.MAX_VALUE;
            long ti = ii < is.size() ? is.get(ii).timeNanos() : Long.MAX_VALUE;
            if (ti <= tw) {
                e.ingest(is.get(ii++));
            } else {
                e.ingest(ws.get(wi++));
            }
        }
    }

    @Test
    void shuffledArrivalYieldsIdenticalTimeline() {
        List<Long> wt = new ArrayList<>();
        List<Long> it = new ArrayList<>();
        long step = 20_000_000L;
        for (int i = 0; i < 60; i++) {
            wt.add(i * step);
            it.add(i * step + 5_000_000L); // 异步：陀螺偏移半拍
        }
        List<WheelSample> wsA = wheels(wt, 1.0);
        List<ImuSample> isA = imus(it, 0.05);

        PoseEstimator ordered = newEstimator();
        feedOrdered(ordered, wsA, isA);

        // 打乱顺序喂入（同一份样本）
        List<Record> all = new ArrayList<>();
        wsA.forEach(w -> all.add(new Record(w, null)));
        isA.forEach(i -> all.add(new Record(null, i)));
        Collections.shuffle(all, new Random(7));

        PoseEstimator shuffled = newEstimator();
        for (Record r : all) {
            if (r.w != null) {
                shuffled.ingest(r.w);
            } else {
                shuffled.ingest(r.i);
            }
        }

        long end = wsA.get(wsA.size() - 1).timeNanos();
        for (long q = 0; q <= end; q += 100_000_000L) {
            PoseEstimate a = ordered.query(q, List.of());
            PoseEstimate b = shuffled.query(q, List.of());
            assertEquals(a.x(), b.x(), 1.0e-5, "x mismatch at " + q);
            assertEquals(a.y(), b.y(), 1.0e-5, "y mismatch at " + q);
            assertEquals(a.theta(), b.theta(), 1.0e-6, "theta mismatch at " + q);
            assertEquals(a.speed(), b.speed(), 1.0e-6, "v mismatch at " + q);
        }
    }

    @Test
    void straightLineMotionMovesForward() {
        PoseEstimator e = newEstimator();
        long step = 20_000_000L;
        int n = 50; // 1 秒
        for (int i = 0; i < n; i++) {
            e.ingest(new WheelSample(i * step, 1.0));
            e.ingest(new ImuSample(i * step, 0.0));
        }
        PoseEstimate end = e.query((n - 1) * step, List.of());
        assertEquals(1.0, end.speed(), 0.05);
        // 1 秒约前进 1m（起步有轻微收敛过程）
        assertTrue(end.x() > 0.8, "应沿 x 正向移动，实际 x=" + end.x());
        assertEquals(0.0, end.y(), 1.0e-6);
        assertEquals(0.0, end.theta(), 1.0e-3);
    }

    @Test
    void outputTimelineIsMonotonic() {
        PoseEstimator e = newEstimator();
        long step = 20_000_000L;
        for (int i = 0; i < 30; i++) {
            e.ingest(new WheelSample(i * step, 1.0));
            if (i >= 10) {
                // 迟到一个窗口内的回拨采样
                e.ingest(new WheelSample((i - 2) * step + 1_000_000L, 1.0));
            }
        }
        long t = e.lastInputTime();
        long prev = -1;
        for (long q = 0; q <= t; q += 10_000_000L) {
            long qt = e.query(q, List.of()).timeNanos();
            assertTrue(qt >= prev, "查询时间轴必须单调");
            prev = qt;
        }
    }

    private record Record(WheelSample w, ImuSample i) {
    }
}
