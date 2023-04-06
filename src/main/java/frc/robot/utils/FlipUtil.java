package frc.robot.utils;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.FieldConstants;

import java.util.ArrayList;
import java.util.List;

import static com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import static edu.wpi.first.math.trajectory.Trajectory.State;

public final class FlipUtil {
    public static double apply(double x) {
        return FieldConstants.FIELD_LENGTH - x;
    }

    public static Translation2d apply(Translation2d translation) {
        return new Translation2d(apply(translation.getX()), translation.getY());
    }

    public static Rotation2d apply(Rotation2d rotation) {
        return new Rotation2d(-rotation.getCos(), rotation.getSin());
    }

    public static Pose2d apply(Pose2d pose) {
        return new Pose2d(apply(pose.getTranslation()), apply(pose.getRotation()));
    }

    public static PathPlannerState apply(PathPlannerTrajectory.PathPlannerState state) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return state;
        }

        Pose2d flippedPose = apply(state.poseMeters);
        Rotation2d flippedHolonomicRotation = apply(state.holonomicRotation);

        var flipped = PathPlannerTrajectory.transformStateForAlliance(state, DriverStation.getAlliance());
        flipped.poseMeters = flippedPose;
        flipped.holonomicRotation = flippedHolonomicRotation;

        return flipped;
    }

    public static PathPlannerTrajectory apply(PathPlannerTrajectory trajectory) {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return trajectory;
        }

        List<State> flippedStates = new ArrayList<>();
        for (State s : trajectory.getStates()) {
            PathPlannerTrajectory.PathPlannerState state = (PathPlannerState) s;
            flippedStates.add(apply(state));
        }

        return new PathPlannerTrajectory(
                flippedStates,
                trajectory.getMarkers(),
                trajectory.getStartStopEvent(),
                trajectory.getEndStopEvent(),
                trajectory.fromGUI
        );
    }
}