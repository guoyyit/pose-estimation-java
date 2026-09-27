package com.github.guoyyit.poseestimationjava.input;

import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.WheelSample;

import java.util.Optional;

/**
 * 本地内存传感器输入接口：所有外部数据经此进入估计器。
 * 实现必须线程安全，支持并发输入与并发查询。
 *
 * @return 非空表示该采样被拒绝及其可区分原因；空表示已接纳
 */
public interface SensorFeeder {
    Optional<AnomalyEvent> pushWheel(WheelSample sample);

    Optional<AnomalyEvent> pushImu(ImuSample sample);
}
