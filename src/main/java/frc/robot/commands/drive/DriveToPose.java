package frc.robot.commands.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.drive.SwerveDrive;

import java.util.function.Supplier;

public class DriveToPose extends CommandBase {

    private final SwerveDrive drive;

    private ProfiledPIDController xController = new ProfiledPIDController(2.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.maxLinearSpeed,
                    Constants.DriveConstants.maxLinearSpeed),
            Constants.loopPeriodSecs);
    private ProfiledPIDController yController = new ProfiledPIDController(2.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.maxLinearSpeed,
                    Constants.DriveConstants.maxLinearSpeed),
            Constants.loopPeriodSecs);

    private ProfiledPIDController rotController = new ProfiledPIDController(Math.PI, 0, 0,
            new TrapezoidProfile.Constraints(
                    Constants.DriveConstants.maxAngularSpeed,
                    Constants.DriveConstants.maxAngularSpeed),
            Constants.loopPeriodSecs);

    private Supplier<Pose2d> goalSupplier;
    private Pose2d goalPose;
    private Supplier<TrapezoidProfile.Constraints> linearConstraintsSupplier;
    private Supplier<TrapezoidProfile.Constraints> angularConstraintsSupplier;

    public DriveToPose(SwerveDrive drive,
                       Supplier<Pose2d> goalSupplier,
                       Supplier<TrapezoidProfile.Constraints> linearConstraintsSupplier,
                       Supplier<TrapezoidProfile.Constraints> angularConstraintsSupplier) {

        this.drive = drive;
        this.goalSupplier = goalSupplier;
        this.linearConstraintsSupplier = linearConstraintsSupplier;
        this.angularConstraintsSupplier = angularConstraintsSupplier;

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        rotController.setTolerance(0.015);

        rotController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(drive);
    }

    public DriveToPose(SwerveDrive drive, Supplier<Pose2d> goalSupplier) {
        this(drive, goalSupplier, null, null);
    }

    @Override
    public void initialize() {
        if (linearConstraintsSupplier != null) {
            xController.setConstraints(linearConstraintsSupplier.get());
            yController.setConstraints(linearConstraintsSupplier.get());
        }
        if (angularConstraintsSupplier != null) {
            rotController.setConstraints(angularConstraintsSupplier.get());
        }

        resetControllers();

        System.out.println("DriveToPose initialize()");
    }

    @Override
    public void execute() {
        if (!goalPose.equals(goalSupplier.get()))
            resetControllers();

        Pose2d currentPose = drive.getPose();

        double vx = xController.calculate(currentPose.getX());
        double vy = yController.calculate(currentPose.getY());
        double vtheta = rotController.calculate(currentPose.getRotation().getRadians());

        drive.drive(vx, vy, vtheta, true);
    }

    @Override
    public boolean isFinished() {
       return (xController.atGoal() && yController.atGoal() && rotController.atGoal());
    }

    @Override
    public void end(boolean interrupted) {
        if (!interrupted)
            drive.drive(0, 0, 0, false);
    }

    private void resetControllers() {
        // set controllers
        goalPose = goalSupplier.get();
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        rotController.setGoal(goalPose.getRotation().getRadians());

        Pose2d currentPose = drive.getPose();
        ChassisSpeeds currentVel = drive.getCurrentVel();
        xController.reset(currentPose.getX(), currentVel.vxMetersPerSecond / 2.0);
        yController.reset(currentPose.getY(), currentVel.vyMetersPerSecond / 2.0);
        rotController.reset(currentPose.getRotation().getRadians(), currentVel.omegaRadiansPerSecond / 2.0);
    }
}