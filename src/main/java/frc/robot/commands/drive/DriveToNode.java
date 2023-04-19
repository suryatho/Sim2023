package frc.robot.commands.drive;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.FlipUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

public class DriveToNode extends DriveTrajectory {
    private final SwerveDrive drive;
    private final Supplier<NodeSelector.Node> nodeSupplier;

    // dist from wall to node plus turn radius and margin
    private final static double intermediateX = Units.inchesToMeters(54.0) + Constants.DriveConstants.turnRadius + 0.15;

    public DriveToNode(SwerveDrive drive, Supplier<NodeSelector.Node> nodeSupplier) {
        super(drive);

        this.drive = drive;
        this.nodeSupplier = nodeSupplier;

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();
        NodeSelector.Node targetNode = nodeSupplier.get();
        List<PathPoint> pathPoints = new ArrayList<>();

        // start point
        pathPoints.add(PathPoint.fromCurrentHolonomicState(currentPose, drive.getCurrentVel()));

        // if we are far from node then we use an indirect path
        final double deltaX = Math.abs(currentPose.getX() - targetNode.scoringPosition.getX());
        final boolean useDirect = deltaX <= (2.60 - (Units.inchesToMeters(54.0) + Constants.robotLength / 2.0));
        if (!useDirect) {
            // using indirect path
            final double firstXPoint = DriverStation.getAlliance() == DriverStation.Alliance.Red?
                    FlipUtil.apply(intermediateX) : intermediateX;
            final Translation2d intermediateTranslation = new Translation2d(firstXPoint, currentPose.getY());

            // get heading
            final Rotation2d intermediateHeading = (currentPose.getX() >= firstXPoint)? new Rotation2d(Math.PI) : new Rotation2d(0.0);

            // get intermediate rotation based on how far along the full trajectory the intermediate translation is
            final double distToIntermediate = currentPose.getTranslation().getDistance(intermediateTranslation);
            final double distFromIntermediateToNode = intermediateTranslation.getDistance(targetNode.scoringPosition.getTranslation());
            // if we are pretty far away make sure to turn all the way
            final double t = (distToIntermediate >= 1.5 ||
                    Math.abs(currentPose.getRotation().getDegrees() - targetNode.scoringPosition.getRotation().getDegrees()) <= 30.0)?
                    1.0 : distToIntermediate / (distToIntermediate + distFromIntermediateToNode);
            final Rotation2d intermediateRotation = currentPose.getRotation().interpolate(targetNode.scoringPosition.getRotation(), t);

            // add point to list
            pathPoints.add(new PathPoint(intermediateTranslation, intermediateHeading, intermediateRotation)
                    .withControlLengths(distToIntermediate / 2.0, 0.01));
        }

        // add last point to list
        pathPoints.add(new PathPoint(
                targetNode.scoringPosition.getTranslation(),
                targetNode.scoringPosition.getRotation(),
                targetNode.scoringPosition.getRotation())
                    .withPrevControlLength(Constants.DriveConstants.turnRadius + 0.15)
        );

        setTrajectory(PathPlanner.generatePath(
                new PathConstraints(Constants.DriveConstants.maxLinearSpeed, Constants.DriveConstants.maxLinearSpeed * 0.6),
                pathPoints
        ));
        super.initialize();
    }
}
