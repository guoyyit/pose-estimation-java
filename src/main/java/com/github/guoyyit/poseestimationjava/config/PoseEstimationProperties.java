package com.github.guoyyit.poseestimationjava.config;

import org.springframework.boot.context.properties.ConfigurationProperties;

/**
 * application.properties 中的可调参数前缀 pose。
 */
@ConfigurationProperties(prefix = "pose")
public class PoseEstimationProperties {
    /** 状态文件路径；为空表示不启用持久化。 */
    private String stateFilePath = "pose-state.json";
    private long randomSeed = 42L;

    public String getStateFilePath() { return stateFilePath; }
    public void setStateFilePath(String stateFilePath) { this.stateFilePath = stateFilePath; }

    public long getRandomSeed() { return randomSeed; }
    public void setRandomSeed(long randomSeed) { this.randomSeed = randomSeed; }
}
