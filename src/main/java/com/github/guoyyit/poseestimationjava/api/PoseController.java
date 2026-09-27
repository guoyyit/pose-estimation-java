package com.github.guoyyit.poseestimationjava.api;

import com.github.guoyyit.poseestimationjava.config.PoseEstimatorConfig;
import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSpeedSample;
import com.github.guoyyit.poseestimationjava.service.PoseEstimatorService;
import org.springframework.http.ResponseEntity;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RestController;

import java.util.List;
import java.util.Map;

/** Thin REST adapter over the in-memory pose estimation service. */
@RestController
@RequestMapping("/api/pose")
public class PoseController {

    private final PoseEstimatorService service;

    public PoseController(PoseEstimatorService service) {
        this.service = service;
    }

    @PostMapping("/wheel")
    public ResponseEntity<Void> wheel(@RequestBody WheelSpeedSample sample) {
        service.submitWheelSpeed(sample);
        return ResponseEntity.accepted().build();
    }

    @PostMapping("/imu")
    public ResponseEntity<Void> imu(@RequestBody ImuSample sample) {
        service.submitImu(sample);
        return ResponseEntity.accepted().build();
    }

    @GetMapping("/current")
    public ResponseEntity<PoseEstimate> current() {
        return service.currentEstimate()
                .map(ResponseEntity::ok)
                .orElseGet(() -> ResponseEntity.noContent().build());
    }

    @GetMapping("/status")
    public Map<String, Object> status() {
        return Map.of("status", service.status().name());
    }

    @GetMapping("/anomalies")
    public List<AnomalyEvent> anomalies() {
        return service.drainAnomalies();
    }

    /** Replaces the estimator configuration for subsequent estimates. */
    @PostMapping("/config")
    public ResponseEntity<Void> config(@RequestBody PoseEstimatorConfig config) {
        service.updateConfig(config);
        return ResponseEntity.accepted().build();
    }
}
