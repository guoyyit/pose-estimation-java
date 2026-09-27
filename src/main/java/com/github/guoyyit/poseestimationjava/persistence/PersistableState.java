package com.github.guoyyit.poseestimationjava.persistence;

import com.github.guoyyit.poseestimationjava.model.PoseState;

/**
 * 持久化落盘内容：最近一次提交的滤波器状态。
 */
public final class PersistableState {
    public PoseState state;

    public PersistableState() {
    }

    public PersistableState(PoseState state) {
        this.state = state;
    }
}
