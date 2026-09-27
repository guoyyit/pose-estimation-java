package com.github.guoyyit.poseestimationjava.web;

import com.github.guoyyit.poseestimationjava.model.AnomalyEvent;
import com.github.guoyyit.poseestimationjava.model.ImuSample;
import com.github.guoyyit.poseestimationjava.model.PoseEstimate;
import com.github.guoyyit.poseestimationjava.model.WheelSample;
import com.github.guoyyit.poseestimationjava.service.PoseEstimationService;
import org.springframework.web.bind.annotation.GetMapping;
import org.springframework.web.bind.annotation.PostMapping;
import org.springframework.web.bind.annotation.RequestBody;
import org.springframework.web.bind.annotation.RequestMapping;
import org.springframework.web.bind.annotation.RequestParam;
import org.springframework.web.bind.annotation.RestController;

import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * 本地 HTTP 接口：内存模拟输入、查询、诊断与持久化。
 */
@RestController
@RequestMapping("/pose")
public class PoseController {

    private final PoseEstimationService service;

    public PoseController(PoseEstimationService service) {
        this.service = service;
    }

    @PostMapping("/input/wheel")
    public Map<String, Object> pushWheel(@RequestBody WheelSample sample) {
        return result(service.pushWheel(sample));
    }

    @PostMapping("/input/imu")
    public Map<String, Object> pushImu(@RequestBody ImuSample sample) {
        return result(service.pushImu(sample));
    }

    private static Map<String, Object> result(Optional<AnomalyEvent> rejected) {
        Map<String, Object> body = new LinkedHashMap<>();
        body.put("accepted", rejected.isEmpty());
        rejected.ifPresent(e -> {
            body.put("anomaly", e.type().name());
            body.put("detail", e.detail());
            body.put("timeNanos", e.timeNanos());
        });
        return body;
    }

    @GetMapping("/latest")
    public PoseEstimate latest() {
        return service.latest();
    }

    @GetMapping("/query")
    public PoseEstimate query(@RequestParam("timeNanos") long timeNanos) {
        return service.queryAt(timeNanos);
    }

    @GetMapping("/anomalies")
    public List<AnomalyEvent> anomalies() {
        return service.anomalies();
    }

    @PostMapping("/persist")
    public Map<String, Object> persist() {
        service.persist();
        return Map.of("persisted", true);
    }
}
