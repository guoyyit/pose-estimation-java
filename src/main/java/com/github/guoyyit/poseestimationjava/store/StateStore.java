package com.github.guoyyit.poseestimationjava.store;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;

import java.util.Optional;

/** Persistence backend for the estimator state (restart recovery). */
public interface StateStore {

    /** Persists the snapshot, replacing any previously stored one. */
    void save(PoseEstimator.PersistedState state);

    /** Loads the most recently persisted snapshot, if any. */
    Optional<PoseEstimator.PersistedState> load();
}
