package com.github.guoyyit.poseestimationjava.store;

import com.github.guoyyit.poseestimationjava.core.PoseEstimator;

import java.io.IOException;
import java.io.UncheckedIOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.List;
import java.util.Optional;

/**
 * File-backed store. Writes are atomic (temp file + rename) so a crash
 * mid-save never corrupts the last good snapshot.
 *
 * <p>Format: one comma-separated line: timestamp, converged, lastGyroZ,
 * lastWheelYawRate, lastWheelTime, lastImuTime, state[5], covariance[25].</p>
 */
public class FileStateStore implements StateStore {

    private final Path file;

    public FileStateStore(Path file) {
        this.file = file;
    }

    @Override
    public synchronized void save(PoseEstimator.PersistedState state) {
        StringBuilder sb = new StringBuilder(640);
        sb.append(state.timestampNanos()).append(',')
                .append(state.converged()).append(',')
                .append(state.lastGyroZ()).append(',')
                .append(state.lastWheelYawRate()).append(',')
                .append(state.lastWheelTimestampNanos()).append(',')
                .append(state.lastImuTimestampNanos());
        for (double v : state.state()) {
            sb.append(',').append(v);
        }
        for (double v : state.covariance()) {
            sb.append(',').append(v);
        }
        sb.append('\n');
        try {
            Path parent = file.toAbsolutePath().getParent();
            if (parent != null) {
                Files.createDirectories(parent);
            }
            Path tmp = file.resolveSibling(file.getFileName() + ".tmp");
            Files.writeString(tmp, sb.toString());
            Files.move(tmp, file, StandardCopyOption.REPLACE_EXISTING,
                    StandardCopyOption.ATOMIC_MOVE);
        } catch (IOException e) {
            throw new UncheckedIOException("failed to persist estimator state to " + file, e);
        }
    }

    @Override
    public synchronized Optional<PoseEstimator.PersistedState> load() {
        if (!Files.exists(file)) {
            return Optional.empty();
        }
        try {
            List<String> lines = Files.readAllLines(file);
            if (lines.isEmpty()) {
                return Optional.empty();
            }
            String[] parts = lines.get(0).trim().split(",");
            int expected = 6 + 5 + 25;
            if (parts.length != expected) {
                return Optional.empty();
            }
            int i = 0;
            long timestamp = Long.parseLong(parts[i++]);
            boolean converged = Boolean.parseBoolean(parts[i++]);
            double lastGyroZ = Double.parseDouble(parts[i++]);
            double lastWheelYawRate = Double.parseDouble(parts[i++]);
            long lastWheelTime = Long.parseLong(parts[i++]);
            long lastImuTime = Long.parseLong(parts[i++]);
            double[] state = new double[5];
            for (int k = 0; k < 5; k++) {
                state[k] = Double.parseDouble(parts[i++]);
            }
            double[] covariance = new double[25];
            for (int k = 0; k < 25; k++) {
                covariance[k] = Double.parseDouble(parts[i++]);
            }
            return Optional.of(new PoseEstimator.PersistedState(timestamp, state, covariance,
                    converged, lastGyroZ, lastWheelYawRate, lastWheelTime, lastImuTime));
        } catch (IOException | RuntimeException e) {
            // A corrupt or unreadable store must not prevent startup.
            return Optional.empty();
        }
    }
}
