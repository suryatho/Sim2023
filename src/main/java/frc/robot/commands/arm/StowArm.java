package frc.robot.commands.arm;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.subsystems.arm.Arm;

import java.util.ArrayList;
import java.util.List;

public class StowArm extends ArmTrajectory {

    private final Arm arm;
    private static final Translation2d stow = Constants.ArmConstants.stowedPosition;
    private static final double directTolerace = 0.30;

    public StowArm(Arm arm) {
        super(arm);
        this.arm = arm;
    }

    @Override
    public void initialize() {
        Pose2d current = arm.getEndPose();

        final boolean useDirect = (current.getTranslation().getDistance(stow) <= directTolerace) &&
                (current.getY() >= stow.getY());

        List<PathPoint> points = new ArrayList<>();
        // direct path
        if (useDirect) {
            System.out.println("Using direct stow trajectory");
            final double distance = current.getTranslation().getDistance(stow);
            final Rotation2d heading = ArmTrajectory.getHeading(current.getTranslation(), stow);
            // start point
            points.add(
                    PathPoint.fromCurrentHolonomicState(current, arm.getGripperSpeed())
                            .withNextControlLength(distance * 0.01)
            );

            // end
            points.add(
                    new PathPoint(stow, heading)
                            .withPrevControlLength(distance)
            );

            setTrajectory(PathPlanner.generatePath(
                    new PathConstraints(4, 4),
                    points
            ));
            super.initialize();
            return;
        }

        // use difference of height to decide control lengths
        final double deltaY = 3.0 - Math.abs(current.getY() - stow.getY());

        points.add(
                new PathPoint(current.getTranslation(), new Rotation2d(Math.PI / 2.0))
                        .withNextControlLength(deltaY)
        );

        points.add(
                new PathPoint(stow, new Rotation2d(-Math.PI / 2.0))
                        .withPrevControlLength(deltaY)
        );

        setTrajectory(PathPlanner.generatePath(
                new PathConstraints(4, 3),
                points
        ));
        super.initialize();
    }
}