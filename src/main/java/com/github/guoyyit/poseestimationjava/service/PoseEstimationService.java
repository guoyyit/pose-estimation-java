package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.estimator.PoseEstimator;
import com.github.guoyyit.poseestimationjava.input.SensorFeeder;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.AnomalyType;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.PoseState;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import com.github.guoyyit.poseestimationjava.persistence.JsonFilePoseStateStore;
import com.github.guoyyit.poseestimationjava.persistence.PersistableState;
import com.github.guoyyit.poseestimationjava.persistence.PoseStateStore;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

/**
 * 位姿估计门面：并发安全的输入、查询、诊断、配置变更与持久化。
 *
 * <h2>并发模型</h2>
 * <p>所有输入与查询都在同一把锁上串行。一次查询构造的是不可变 {@link PoseEstimate}
 * 快照，读取到的位姿必对应某个确定时间点，绝不会读到半更新状态。</p>
 *
 * <h2>异常事件</h2>
 * <p>被拒绝的采样（重复、回拨、过期、轮速突变、零偏跳变）进入有界历史列表；
 * “长时间无输入”按查询时间显式判定（{@link AnomalyType#INPUT_TIMEOUT}），
 * 不使用墙钟，因此回放完全确定。</p>
 *
 * <h2>配置变更</h2>
 * <p>{@link #updateConfig} 原子替换配置引用，估计器从下一采样起采用新参数，
 * 已提交状态不变，不产生跳变。</p>
 *
 * <h2>重启恢复</h2>
 * <p>构造时若状态文件存在则恢复最近检查点；{@link #persist} 原子写盘。
 * 恢复后的时间轴与重启前连续。</p>
 */
public final class PoseEstimationService implements SensorFeeder {

    private static final int MAX_EVENT_HISTORY = 512;

    private final Object lock = new Object();
    private PoseConfig config;
    private PoseEstimator estimator;
    private final PoseStateStore store;
    private final List<AnomalyEvent> eventHistory = new ArrayList<>();

    public PoseEstimationService() {
        this(PoseConfig.defaults(), null);
    }

    public PoseEstimationService(PoseConfig config, String stateFilePath) {
        this.config = config;
        this.estimator = new PoseEstimator(config);
        this.store = stateFilePath == null || stateFilePath.isBlank()
                ? null : new JsonFilePoseStateStore(stateFilePath);
        if (this.store != null) {
            this.store.load().ifPresent(p -> {
                PoseState s = p.state;
                // 恢复不产生跳变：以落盘检查点为估计起点
                estimator.restore(s);
            });
        }
    }

    @Override
    public Optional<AnomalyEvent> pushWheel(WheelSample sample) {
        if (sample == null) {
            return Optional.empty();
        }
        synchronized (lock) {
            Optional<AnomalyEvent> rejected = estimator.ingest(sample);
            record(rejected);
            return rejected;
        }
    }

    @Override
    public Optional<AnomalyEvent> pushImu(ImuSample sample) {
        if (sample == null) {
            return Optional.empty();
        }
        synchronized (lock) {
            Optional<AnomalyEvent> rejected = estimator.ingest(sample);
            record(rejected);
            return rejected;
        }
    }

    private void record(Optional<AnomalyEvent> event) {
        event.ifPresent(e -> {
            eventHistory.add(e);
            while (eventHistory.size() > MAX_EVENT_HISTORY) {
                eventHistory.remove(0);
            }
        });
    }

    /** 最近一个已确定时间点（最新采样时间）的输出。 */
    public PoseEstimate latest() {
        synchronized (lock) {
            if (!estimator.hasData()) {
                return null;
            }
            return estimator.query(estimator.lastInputTime(), List.of());
        }
    }

    /**
     * 指定时间点查询。时间点早于首个采样返回 null；晚于最新采样时沿模型外推，
     * 若外推跨度超过超时阈值则附加 INPUT_TIMEOUT 事件。
     */
    public PoseEstimate queryAt(long timeNanos) {
        synchronized (lock) {
            if (!estimator.hasData()) {
                return null;
            }
            long last = estimator.lastInputTime();
            List<AnomalyEvent> extra = List.of();
            if (timeNanos - last > config.inputTimeoutNanos) {
                extra = List.of(new AnomalyEvent(AnomalyType.INPUT_TIMEOUT, timeNanos,
                        "no input for " + (timeNanos - last) + " ns (>"
                                + config.inputTimeoutNanos + ")"));
            }
            return estimator.query(timeNanos, extra);
        }
    }

    public PoseConfig currentConfig() {
        synchronized (lock) {
            return config;
        }
    }

    /** 原子更新配置；仅影响后续估计，已提交状态连续不变。 */
    public void updateConfig(PoseConfig newConfig) {
        synchronized (lock) {
            this.config = newConfig;
            this.estimator.updateConfig(newConfig);
        }
    }

    /** 显式持久化最近检查点。 */
    public void persist() {
        synchronized (lock) {
            if (store == null) {
                return;
            }
            PoseState snapshot = estimator.snapshot();
            if (snapshot != null) {
                store.save(new PersistableState(snapshot));
            }
        }
    }

    /** 异常事件历史（最近 MAX_EVENT_HISTORY 条）。 */
    public List<AnomalyEvent> anomalies() {
        synchronized (lock) {
            List<AnomalyEvent> all = new ArrayList<>(eventHistory);
            all.addAll(estimator.windowEvents());
            return List.copyOf(all);
        }
    }

    /** 清空全部状态与持久化文件（测试用）。 */
    public void reset() {
        synchronized (lock) {
            eventHistory.clear();
            if (store != null) {
                store.clear();
            }
            estimator.reset();
        }
    }
}
