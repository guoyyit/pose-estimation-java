package com.github.guoyyit.poseestimationjava.service;

import org.springframework.scheduling.annotation.EnableScheduling;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

/**
 * Drives the estimator clock in real deployments: advances the reorder
 * horizon and runs the no-input watchdog. Sample timestamps are expected to
 * be in the same clock domain as {@link System#nanoTime()}.
 */
@Component
@EnableScheduling
public class PoseEstimatorClock {

    private final PoseEstimatorService service;

    public PoseEstimatorClock(PoseEstimatorService service) {
        this.service = service;
    }

    @Scheduled(fixedRateString = "${pose.clock-interval-ms:100}")
    public void tick() {
        service.onTimer(System.nanoTime());
    }
}
