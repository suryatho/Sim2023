package frc.robot.utils.poseEstimator;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

import java.util.Map;
import java.util.NavigableMap;
import java.util.TreeMap;

/** Mechanical Advantage implementation of pose estimator */
public class PoseEstimator2 {
    private Pose2d basePose = new Pose2d();
    private Map.Entry<Double, Pose2d> currentPoseEstimateEntry = Map.entry(0.0, new Pose2d());
    private final NavigableMap<Double, PoseUpdate> updates = new TreeMap<>();
    private final Matrix<N3, N1> stateStdDevs = new Matrix<>(Nat.N3(), Nat.N1());

    private static final double kHistoryLengthSeconds = 0.3;

    public PoseEstimator2(Matrix<N3, N1> stateStdDevs) {
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
        if (updates.size() < 1)
            return basePose;
        Pose2d estimatedPose;
        if (currentPoseEstimateEntry.getKey() <= updates.firstKey()) {
            // go through all updates
            estimatedPose = basePose;
            for (var update : updates.entrySet()) {
                estimatedPose = update.getValue().apply(estimatedPose);
            }
        } else if (currentPoseEstimateEntry.getKey() < updates.lastKey()) {
            // go through all updates after current estimate
            estimatedPose = currentPoseEstimateEntry.getValue();
            var shortenedUpdates = updates.tailMap(currentPoseEstimateEntry.getKey(), false);
            for (var update : shortenedUpdates.entrySet()) {
                estimatedPose = update.getValue().apply(estimatedPose);
            }
        } else {
            // no recent updates so return current estimate
            estimatedPose = currentPoseEstimateEntry.getValue();
        }
        currentPoseEstimateEntry = Map.entry(updates.lastKey(), estimatedPose);
        return estimatedPose;
    }

    private void update() {
        // update the base pose
        while (updates.size() > 1 &&
                updates.firstKey() < Timer.getFPGATimestamp() - kHistoryLengthSeconds) {
            var update = updates.pollFirstEntry();
            basePose = update.getValue().apply(basePose);
        }
    }

    public void resetPose(Pose2d pose) {
        basePose = pose;
        updates.clear();
        currentPoseEstimateEntry = Map.entry(0.0, pose);
    }

    private static class PoseUpdate {
        public final Twist2d driveTwist;

        public PoseUpdate(Twist2d driveTwist) {
            this.driveTwist = driveTwist;
        }

        public Pose2d apply(Pose2d basePose) {
            return basePose.exp(driveTwist);
        }
    }
}