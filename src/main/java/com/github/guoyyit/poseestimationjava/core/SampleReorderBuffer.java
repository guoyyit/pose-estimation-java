package com.github.guoyyit.poseestimationjava.core;

import com.github.guoyyit.poseestimationjava.model.SensorType;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Set;

/**
 * Timestamp-ordered buffer that sorts asynchronously arriving samples before
 * the estimator consumes them. Ordering key is (timestamp, arrival sequence)
 * so samples with equal timestamps keep arrival order and the output timeline
 * is monotonic. Exact duplicates (same type and timestamp) are dropped.
 */
final class SampleReorderBuffer {

    /** A single buffered sample, unified across sensor types. */
    record BufferedSample(long timestampNanos,
                          SensorType type,
                          double v1,
                          double v2,
                          double v3,
                          long sequence) {
    }

    private final PriorityQueue<BufferedSample> queue;
    /** Dedup keys of samples currently in the buffer: type + timestamp. */
    private final Set<Long> keys = new HashSet<>();

    SampleReorderBuffer() {
        // Deterministic total order: timestamp, then sensor type, then arrival
        // sequence. The same multiset of samples therefore always produces the
        // same processing order regardless of arrival permutation.
        this.queue = new PriorityQueue<>((a, b) -> {
            int c = Long.compare(a.timestampNanos, b.timestampNanos);
            if (c != 0) {
                return c;
            }
            c = Integer.compare(a.type.ordinal(), b.type.ordinal());
            return c != 0 ? c : Long.compare(a.sequence, b.sequence);
        });
    }

    private static long key(BufferedSample s) {
        // Distinct key space per sensor type, then timestamp.
        return 31L * s.type.ordinal() + s.timestampNanos;
    }

    /**
     * Offers a sample.
     *
     * @return true if buffered, false if it was an exact duplicate
     */
    boolean offer(BufferedSample sample) {
        if (!keys.add(key(sample))) {
            return false;
        }
        queue.add(sample);
        return true;
    }

    /**
     * Removes and returns all samples with timestamp <= horizon, in
     * (timestamp, sequence) order.
     */
    List<BufferedSample> drainUpTo(long horizonNanos) {
        List<BufferedSample> out = new ArrayList<>();
        while (!queue.isEmpty() && queue.peek().timestampNanos <= horizonNanos) {
            BufferedSample s = queue.poll();
            keys.remove(key(s));
            out.add(s);
        }
        return out;
    }

    /** Removes and returns every buffered sample in order. */
    List<BufferedSample> drainAll() {
        return drainUpTo(Long.MAX_VALUE);
    }

    int size() {
        return queue.size();
    }
}
