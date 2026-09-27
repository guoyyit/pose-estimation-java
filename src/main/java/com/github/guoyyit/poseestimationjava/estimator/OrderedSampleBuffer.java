package com.github.guoyyit.poseestimationjava.estimator;

import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyType;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.WheelSample;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * 按数据流分别缓存原始采样并始终保持时间升序，支持异步、乱序到达。
 *
 * <h2>接纳规则（原因可区分）</h2>
 * <ul>
 *   <li>同一数据流出现完全相同时间戳：拒绝，{@link AnomalyType#DUPLICATE_TIMESTAMP}。</li>
 *   <li>时间戳早于已提交检查点减去重处理窗口：拒绝，{@link AnomalyType#STALE_SAMPLE}。</li>
 *   <li>时间戳小于该数据流已接受的最大时间戳（但仍在窗口内，即“回拨/迟到”）：
 *       接纳并入序，重处理时并入时间轴，同时给出
 *       {@link AnomalyType#TIMESTAMP_ROLLBACK} 诊断事件——接纳保证连续性，事件保留原因。</li>
 *   <li>其余正常按序接纳。</li>
 * </ul>
 *
 * <p>本类自身不加锁；线程安全由 {@code PoseEstimationService} 的单把锁保证。</p>
 */
public final class OrderedSampleBuffer {

    private final List<WheelSample> wheels = new ArrayList<>();
    private final List<ImuSample> imus = new ArrayList<>();
    private long horizonNanos;
    /** 已提交（已对外形成连续输出）的时间点；早于 base-horizon 的采样无法并入。 */
    private long baseTimeNanos = Long.MIN_VALUE;
    private boolean baseSet = false;

    public OrderedSampleBuffer(long horizonNanos) {
        this.horizonNanos = horizonNanos;
    }

    public void setHorizonNanos(long horizonNanos) {
        this.horizonNanos = horizonNanos;
    }

    public AddOutcome addWheel(WheelSample sample) {
        synchronized (wheels) {
            return add(wheels, sample.timeNanos(), "wheel",
                    new WheelSample(sample.timeNanos(), sample.speed()));
        }
    }

    public AddOutcome addImu(ImuSample sample) {
        synchronized (imus) {
            return add(imus, sample.timeNanos(), "imu",
                    new ImuSample(sample.timeNanos(), sample.gyroZ()));
        }
    }

    /**
     * 入序结果。
     *
     * @param accepted 是否被接纳进入时间轴（false 表示丢弃）
     * @param event    拒绝原因，或接纳但需报告的“回拨/迟到”诊断事件
     */
    public record AddOutcome(boolean accepted, AnomalyEvent event) {
        public boolean isAccepted() {
            return accepted;
        }

        static AddOutcome acceptedOk() {
            return new AddOutcome(true, null);
        }

        static AddOutcome acceptedWithRollback(AnomalyEvent e) {
            return new AddOutcome(true, e);
        }

        static AddOutcome rejected(AnomalyEvent e) {
            return new AddOutcome(false, e);
        }
    }

    /**
     * 泛型入序：{@code value} 已构造好；类型仅用于读取时间戳。
     */
    private <T> AddOutcome add(List<T> list, long t, String stream, T value) {
        if (containsTimestamp(list, t)) {
            return AddOutcome.rejected(new AnomalyEvent(
                    AnomalyType.DUPLICATE_TIMESTAMP, t,
                    stream + " duplicate timestamp " + t));
        }
        if (baseSet && t < baseTimeNanos - horizonNanos) {
            return AddOutcome.rejected(new AnomalyEvent(
                    AnomalyType.STALE_SAMPLE, t,
                    stream + " sample " + t + " older than horizon base " + baseTimeNanos));
        }
        long maxSeen = list.isEmpty() ? Long.MIN_VALUE : timestamp(list.get(list.size() - 1));
        list.add(insertionIndex(list, t), value);
        if (maxSeen != Long.MIN_VALUE && t < maxSeen) {
            // 接纳并入序，但报告“回拨/迟到”，供重处理吸收且原因可区分
            return AddOutcome.acceptedWithRollback(new AnomalyEvent(
                    AnomalyType.TIMESTAMP_ROLLBACK, t,
                    stream + " out-of-order sample " + t + " merged (previous max "
                            + maxSeen + ")"));
        }
        return AddOutcome.acceptedOk();
    }

    /** 找第一个时间戳 &gt;= t 的位置（下界）。 */
    private static <T> int insertionIndex(List<T> list, long t) {
        int lo = 0;
        int hi = list.size();
        while (lo < hi) {
            int mid = (lo + hi) >>> 1;
            if (timestamp(list.get(mid)) < t) {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }
        return lo;
    }

    private static long timestamp(Object o) {
        if (o instanceof WheelSample w) {
            return w.timeNanos();
        }
        return ((ImuSample) o).timeNanos();
    }

    private static boolean containsTimestamp(List<?> list, long t) {
        for (Object o : list) {
            if (timestamp(o) == t) {
                return true;
            }
        }
        return false;
    }

    public List<WheelSample> wheels() {
        synchronized (wheels) {
            return List.copyOf(wheels);
        }
    }

    public List<ImuSample> imus() {
        synchronized (imus) {
            return List.copyOf(imus);
        }
    }

    public Optional<Long> maxAcceptedTime() {
        Long w = wheels.isEmpty() ? null : wheels.get(wheels.size() - 1).timeNanos();
        Long i = imus.isEmpty() ? null : imus.get(imus.size() - 1).timeNanos();
        if (w == null) {
            return Optional.ofNullable(i);
        }
        if (i == null) {
            return Optional.of(w);
        }
        return Optional.of(Math.max(w, i));
    }

    /** 撤回最近加入的轮速采样（内容预检失败时使用）。 */
    public void removeWheelAt(long timeNanos) {
        synchronized (wheels) {
            wheels.removeIf(s -> s.timeNanos() == timeNanos);
        }
    }

    /** 撤回最近加入的惯性采样（内容预检失败时使用）。 */
    public void removeImuAt(long timeNanos) {
        synchronized (imus) {
            imus.removeIf(s -> s.timeNanos() == timeNanos);
        }
    }

    /** 缓冲中最晚轮速（内容预检基准）。 */
    public Optional<WheelSample> latestWheel() {
        synchronized (wheels) {
            return wheels.isEmpty() ? Optional.empty()
                    : Optional.of(wheels.get(wheels.size() - 1));
        }
    }

    /** 两路采样中的最小时间戳。 */
    public Optional<Long> minAcceptedTime() {
        Long w = wheels.isEmpty() ? null : wheels.get(0).timeNanos();
        Long i = imus.isEmpty() ? null : imus.get(0).timeNanos();
        if (w == null) {
            return Optional.ofNullable(i);
        }
        if (i == null) {
            return Optional.of(w);
        }
        return Optional.of(Math.min(w, i));
    }

    /** 登记检查点并裁剪早于 (baseTime - horizon) 的采样。 */
    public void prune(long baseTimeNanos) {
        this.baseTimeNanos = baseTimeNanos;
        this.baseSet = true;
        long cutoff = baseTimeNanos - horizonNanos;
        synchronized (wheels) {
            wheels.removeIf(s -> s.timeNanos() < cutoff);
        }
        synchronized (imus) {
            imus.removeIf(s -> s.timeNanos() < cutoff);
        }
    }

    public void reset() {
        wheels.clear();
        imus.clear();
        baseSet = false;
        baseTimeNanos = Long.MIN_VALUE;
    }
}
