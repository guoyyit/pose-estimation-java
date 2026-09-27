package com.github.guoyyit.poseestimationjava.service;

import com.github.guoyyit.poseestimationjava.config.PoseConfig;
import com.github.guoyyit.poseestimationjava.config.PoseEstimationProperties;
import org.springframework.boot.context.properties.EnableConfigurationProperties;
import org.springframework.context.annotation.Bean;
import org.springframework.context.annotation.Configuration;

/**
 * Spring 装配：默认参数、随机种子与状态文件路径来自配置。
 */
@Configuration
@EnableConfigurationProperties(PoseEstimationProperties.class)
public class PoseServiceConfig {

    @Bean
    public PoseEstimationService poseEstimationService(PoseEstimationProperties properties) {
        return new PoseEstimationService(PoseConfig.defaults(), properties.getStateFilePath());
    }
}
