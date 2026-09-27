package com.github.guoyyit.poseestimationjava.persistence;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.github.guoyyit.poseestimationjava.model.PoseState;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.Optional;

/**
 * 基于本地 JSON 文件的状态存储。
 *
 * <p>写入采用“临时文件 + 原子 rename”，重启读到的状态要么完整要么不存在，
 * 不会读到半写入文件。状态内容仅含最近一次提交检查点
 * （位姿、速度、零偏、5×5 协方差、时间戳），恢复后估计器以该检查点为起点，
 * 输出时间轴与重启前连续、不产生跳变。</p>
 */
public final class JsonFilePoseStateStore implements PoseStateStore {

    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final Path filePath;
    private final Path tempPath;

    public JsonFilePoseStateStore(String filePath) {
        this.filePath = Path.of(filePath);
        this.tempPath = Path.of(filePath + ".tmp");
    }

    @Override
    public synchronized void save(PersistableState state) {
        try {
            if (filePath.getParent() != null && !Files.exists(filePath.getParent())) {
                Files.createDirectories(filePath.getParent());
            }
            byte[] bytes = MAPPER.writerWithDefaultPrettyPrinter()
                    .writeValueAsBytes(new StateDto(state.state));
            Files.write(tempPath, bytes);
            // 先写完整临时文件再替换目标：读者要么看到旧文件要么看到完整新文件，
            // 不会读到半写入状态。先尝试原子 rename，不支持时退化为替换。
            try {
                Files.move(tempPath, filePath, StandardCopyOption.ATOMIC_MOVE);
            } catch (UnsupportedOperationException | java.nio.file.AtomicMoveNotSupportedException
                    | java.nio.file.FileAlreadyExistsException ex) {
                Files.move(tempPath, filePath, StandardCopyOption.REPLACE_EXISTING);
            }
        } catch (IOException e) {
            throw new UncheckedIOException("failed to persist pose state to " + filePath, e);
        }
    }

    @Override
    public synchronized Optional<PersistableState> load() {
        if (!Files.exists(filePath)) {
            return Optional.empty();
        }
        try {
            StateDto dto = MAPPER.readValue(Files.readAllBytes(filePath), StateDto.class);
            PoseState s = new PoseState();
            s.x = dto.x;
            s.y = dto.y;
            s.theta = dto.theta;
            s.v = dto.v;
            s.bias = dto.bias;
            s.timeNanos = dto.timeNanos;
            if (dto.covariance != null && dto.covariance.length == s.covariance.length) {
                System.arraycopy(dto.covariance, 0, s.covariance, 0, s.covariance.length);
            }
            return Optional.of(new PersistableState(s));
        } catch (IOException e) {
            throw new UncheckedIOException("failed to load pose state from " + filePath, e);
        }
    }

    @Override
    public synchronized void clear() {
        try {
            Files.deleteIfExists(filePath);
            Files.deleteIfExists(tempPath);
        } catch (IOException e) {
            throw new UncheckedIOException("failed to clear pose state at " + filePath, e);
        }
    }

    /** 序列化 DTO：字段名稳定即格式契约。 */
    public static final class StateDto {
        public long timeNanos;
        public double x;
        public double y;
        public double theta;
        public double v;
        public double bias;
        public double[] covariance;

        public StateDto() {
        }

        StateDto(PoseState s) {
            this.timeNanos = s.timeNanos;
            this.x = s.x;
            this.y = s.y;
            this.theta = s.theta;
            this.v = s.v;
            this.bias = s.bias;
            this.covariance = s.covariance.clone();
        }
    }
}
