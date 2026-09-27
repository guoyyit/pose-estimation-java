package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.core.PoseEstimator;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.EstimatorStatus;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;
import com.github.guoyyit.poseestimationjava.store.FileStateStore;
import com.github.guoyyit.poseestimationjava.store.StateStore;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Service;

import java.nio.file.Path;
import java.util.List;
import java.util.Optional;

/**
 * Thread-safe in-memory entry point of the pose estimation capability.
 * Inputs arrive through plain method calls (local in-memory interface).
 *
 * <p>The committed estimator state is persisted after every input so a
 * restart resumes from the most recent snapshot without a pose jump.</p>
 */
@Service
public class PoseEstimatorService {

    private final PoseEstimator estimator;
    private final StateStore store;

    @Autowired
    public PoseEstimatorService(
            @Value("${pose.state-file:pose-state.txt}") String stateFile) {
        this(PoseEstimatorConfig.defaults(), new FileStateStore(Path.of(stateFile)));
    }

    public PoseEstimatorService(PoseEstimatorConfig config, StateStore store) {
        this.store = store;
        this.estimator = store.load()
                .map(s -> new PoseEstimator(config, s))
                .orElseGet(() -> new PoseEstimator(config));
    }

    public void submitWheelSpeed(WheelSpeedSample sample) {
        estimator.submitWheelSpeed(sample);
        persist();
    }

    public void submitImu(ImuSample sample) {
        estimator.submitImu(sample);
        persist();
    }

    /** Advances the estimator clock (no-input watchdog, buffer flushing). */
    public void onTimer(long nowNanos) {
        estimator.onTimer(nowNanos);
        persist();
    }

    /** Forces processing of all buffered samples (end of input burst). */
    public void flush() {
        estimator.flush();
        persist();
    }

    public Optional<PoseEstimate> currentEstimate() {
        return estimator.currentEstimate();
    }

    public List<AnomalyEvent> drainAnomalies() {
        return estimator.drainAnomalies();
    }

    public EstimatorStatus status() {
        return estimator.status();
    }

    /** Applies a new configuration to subsequent estimates. */
    public void updateConfig(PoseEstimatorConfig config) {
        estimator.updateConfig(config);
    }

    private void persist() {
        PoseEstimator.PersistedState state = estimator.persistableState();
        if (state.timestampNanos() >= 0) {
            store.save(state);
        }
    }
}
