package com.github.guoyyit.poseestimationjava.store;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicReference;

/** Volatile in-memory store, mainly for tests. */
public class InMemoryStateStore implements StateStore {

    private final AtomicReference<PoseEstimator.PersistedState> slot = new AtomicReference<>();

    @Override
    public void save(PoseEstimator.PersistedState state) {
        slot.set(state);
    }

    @Override
    public Optional<PoseEstimator.PersistedState> load() {
        return Optional.ofNullable(slot.get());
    }
}
