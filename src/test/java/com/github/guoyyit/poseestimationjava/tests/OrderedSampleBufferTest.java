package com.github.guoyyit.poseestimationjava.tests;

import com.github.guoyyit.poseestimationjava.estimator.OrderedSampleBuffer;
import com.github.guoyyit.poseestimationjava.model.AnomalyType;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

class OrderedSampleBufferTest {

    private static final long HORIZON = 1_000_000L;

    @Test
    void outOfOrderSamplesAreSortedIntoTimeline() {
        OrderedSampleBuffer buf = new OrderedSampleBuffer(HORIZON);
        // 乱序到达：均被接纳
        assertTrue(buf.addWheel(new WheelSample(30, 3.0)).isAccepted());
        assertTrue(buf.addWheel(new WheelSample(10, 1.0)).isAccepted());
        assertTrue(buf.addWheel(new WheelSample(20, 2.0)).isAccepted());

        var wheels = buf.wheels();
        assertEquals(3, wheels.size());
        assertEquals(10, wheels.get(0).timeNanos());
        assertEquals(20, wheels.get(1).timeNanos());
        assertEquals(30, wheels.get(2).timeNanos());
    }

    @Test
    void duplicateTimestampRejectedWithDistinctReason() {
        OrderedSampleBuffer buf = new OrderedSampleBuffer(HORIZON);
        assertTrue(buf.addImu(new ImuSample(100, 0.1)).isAccepted());
        var dup = buf.addImu(new ImuSample(100, 0.2));
        assertFalse(dup.isAccepted());
        assertEquals(AnomalyType.DUPLICATE_TIMESTAMP, dup.event().type());
        assertEquals(1, buf.imus().size());
    }

    @Test
    void rollbackWithinHorizonAcceptedButReported() {
        OrderedSampleBuffer buf = new OrderedSampleBuffer(HORIZON);
        buf.addWheel(new WheelSample(500, 1.0));
        var late = buf.addWheel(new WheelSample(400, 0.5));
        // 接纳（保证连续）但带回拨事件（原因可区分）
        assertTrue(late.isAccepted());
        assertEquals(AnomalyType.TIMESTAMP_ROLLBACK, late.event().type());
        assertEquals(2, buf.wheels().size());
        assertEquals(400, buf.wheels().get(0).timeNanos());
    }

    @Test
    void staleSampleBeyondHorizonRejected() {
        OrderedSampleBuffer buf = new OrderedSampleBuffer(HORIZON);
        buf.prune(1_500_000L);
        var stale = buf.addWheel(new WheelSample(400_000L, 1.0)); // < base-horizon=500_000
        assertFalse(stale.isAccepted());
        assertEquals(AnomalyType.STALE_SAMPLE, stale.event().type());
        assertEquals(0, buf.wheels().size());

        // 窗口内仍可接纳且无事件
        var ok = buf.addWheel(new WheelSample(600_000L, 1.0));
        assertTrue(ok.isAccepted());
        assertNull(ok.event());
        assertEquals(1, buf.wheels().size());
    }
}
