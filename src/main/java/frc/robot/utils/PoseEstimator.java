package frc.robot.utils;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.*;

/** Mechanical Advantage implementation of pose estimator */
public class PoseEstimator {
    private Pose2d basePose = new Pose2d();
    private Pose2d latestPose = new Pose2d();
    private double latestTime = 0.0;
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> stateStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private static final double kHistoryLengthSeconds = 0.3;

    public PoseEstimator(Matrix<N3, N1> stateStdDevs) {
        for (int i = 0; i < 3; i++) {
            this.stateStdDevs.set(i, 0, stateStdDevs.get(i, 0) * stateStdDevs.get(i, 0));
        }
    }

    public void addDriveTwist(double timestamp, Twist2d twist) {
        updates.put(timestamp, new PoseUpdate(twist));
        update();
    }

    public Pose2d getEstimatedPose() {
        update();

        // if there are no updates
        if (updates.size() < 1) {
            latestPose = basePose;
            return latestPose;
        }

        // if the latest pose is from before the first update
        if (latestTime <= updates.firstKey()) {
            // go through all updates
            latestPose = basePose;
            for (var update : updates.entrySet()) {
                latestPose = update.getValue().apply(latestPose, stateStdDevs);
            }
        } else if (latestTime < updates.lastKey()) {
            // go through all updates after current estimate
            var shortenedUpdates = updates.tailMap(latestTime, false);
            for (var update : shortenedUpdates.entrySet()) {
                latestPose = update.getValue().apply(latestPose, stateStdDevs);
            }
        }

        // update the time of the latest pose
        latestTime = updates.lastKey();

        // if there are no recent updates so return current estimate
        return latestPose;
    }

    private void update() {
        // update the base pose
        while (updates.size() > 1 &&
                updates.firstKey() < Timer.getFPGATimestamp() - kHistoryLengthSeconds) {
            var update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose, stateStdDevs);
        }
    }

    public void resetPose(Pose2d pose) {
        basePose = pose;
        latestPose = pose;
        latestTime = 0.0;
        updates.clear();
    }

    private static class PoseUpdate {
        public final Twist2d driveTwist;
        public final List<ExternalMeasurement> externalMeasurements = new ArrayList<>();

        public PoseUpdate(Twist2d driveTwist) {
            this.driveTwist = driveTwist;
        }

        public Pose2d apply(Pose2d pose, Matrix<N3, N1> stateStdDevs) {
            externalMeasurements.sort(ExternalMeasurement::compareTo);
            return pose.exp(driveTwist);
        }
    }

    public static class ExternalMeasurement implements Comparable<ExternalMeasurement> {
        public final Pose2d pose;
        public final Matrix<N3, N1> stdDevs;

        public ExternalMeasurement(Pose2d pose, Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.stdDevs = new Matrix<>(Nat.N3(), Nat.N1());
            for (int row = 0; row < 3; row++) {
                this.stdDevs.set(row, 0, stdDevs.get(row, 0));
            }
        }

        /** compares measurements in descending order based on std devs */
        @Override
        public int compareTo(ExternalMeasurement o) {
            return (int) -Math.signum(stdDevs.elementSum() - o.stdDevs.elementSum());
        }
    }
}