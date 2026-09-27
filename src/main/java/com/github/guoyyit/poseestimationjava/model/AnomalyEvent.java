package com.github.guoyyit.poseestimationjava.model;

/**
 * 一次异常事件。
 *
 * @param type      异常类型
 * @param timeNanos 异常对应的采样时间戳
 * @param detail    人类可读的区分信息
 */
public record AnomalyEvent(AnomalyType type, long timeNanos, String detail) {
}
