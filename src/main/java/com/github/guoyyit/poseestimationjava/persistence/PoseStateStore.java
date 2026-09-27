package com.github.guoyyit.poseestimationjava.persistence;

import java.util.Optional;

/**
 * 状态持久化接口（本地）。
 */
public interface PoseStateStore {
    void save(PersistableState state);

    Optional<PersistableState> load();

    void clear();
}
