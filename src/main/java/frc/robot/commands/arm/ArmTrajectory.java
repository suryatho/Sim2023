package frc.robot.commands.arm;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

public class ArmTrajectory extends CommandBase {

    private PPHolonomicDriveController controller = new PPHolonomicDriveController(
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(0.0, 0.0, 0.0)
    );

    private final Arm arm;

    private Supplier<PathPlannerTrajectory> trajectorySupplier;
    private PathPlannerTrajectory trajectory;

    private final Timer timer = new Timer();

    public ArmTrajectory(Arm arm, Supplier<PathPlannerTrajectory> trajectorySupplier) {
        this.arm = arm;
        setTrajectory(trajectorySupplier);

        controller.setTolerance(new Pose2d(0.03, 0.03, new Rotation2d(2 * Math.PI)));

        addRequirements(arm);
    }

    public ArmTrajectory(Arm arm, PathPlannerTrajectory trajectory) {
        this(arm, () -> trajectory);
    }

    public ArmTrajectory(Arm arm) {
        this(arm, () -> null);
    }

    public void setTrajectory(Supplier<PathPlannerTrajectory> trajectorySupplier) {
        this.trajectorySupplier = trajectorySupplier;
    }

    public void setTrajectory(PathPlannerTrajectory trajectory) {
        trajectorySupplier = () -> trajectory;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        trajectory = trajectorySupplier.get();
    }

    @Override
    public void execute() {
        var setpoint = (PathPlannerTrajectory.PathPlannerState) trajectory.sample(timer.get());
        Pose2d current = arm.getEndPose();

        var speeds = controller.calculate(current, setpoint);
        arm.moveEnd(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= trajectory.getTotalTimeSeconds() && controller.atReference()) ||
                (timer.get() >= trajectory.getTotalTimeSeconds() + 1.0);
    }

    @Override
    public void end(boolean interrupted) {
        arm.moveEnd(0, 0);
    }
}
