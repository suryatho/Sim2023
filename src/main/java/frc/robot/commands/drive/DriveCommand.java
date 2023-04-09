package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.drive.SwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveCommand extends CommandBase {

    private final SwerveDrive drive;

    private DoubleSupplier txSupp, tySupp, omegaSupp;
    private BooleanSupplier fieldRelative;
    private DoubleSupplier turbo;

    private DoubleSupplier robotAngleSupplier;
    private ProfiledPIDController rotController = new ProfiledPIDController(
            1.0 / Math.PI, 0.0, 0.0,
            new TrapezoidProfile.Constraints(8 * Math.PI, 6 * Math.PI));

    private boolean useSetpointRotMode = false;

    private static final double minSpeed = 0.3;
    private double speedScale = minSpeed;

    private SlewRateLimiter txLimiter = new SlewRateLimiter(3.0),
    tyLimiter = new SlewRateLimiter(3.0), omegaLimiter = new SlewRateLimiter(3.0);

    private boolean isRed = false;

    public DriveCommand(SwerveDrive drive,
                        DoubleSupplier txSupp,
                        DoubleSupplier tySupp,
                        DoubleSupplier omegaSupp,
                        BooleanSupplier fieldRelative,
                        DoubleSupplier turbo) {
        this.drive = drive;

        this.txSupp = () -> signedSquare(txSupp.getAsDouble());
        this.tySupp = () -> signedSquare(tySupp.getAsDouble());;
        this.omegaSupp = () -> signedSquare(omegaSupp.getAsDouble());;

        this.fieldRelative = fieldRelative;
        this.turbo = turbo;

        this.robotAngleSupplier = () -> drive.getOdometryPose().getRotation().getRadians();

        rotController.enableContinuousInput(-Math.PI, Math.PI);
        rotController.setTolerance((0.5 / 360.0) * 2 * Math.PI);

        addRequirements(drive);
    }

    @Override
    public void initialize() {
        useSetpointRotMode = false;
        rotController.setGoal(0.0);

        isRed = DriverStation.getAlliance() == DriverStation.Alliance.Red;

        txLimiter.reset(txSupp.getAsDouble());
        tyLimiter.reset(tySupp.getAsDouble());
        omegaLimiter.reset(omegaSupp.getAsDouble());

        double dRadians = drive.getCurrentTwist().dtheta;
        rotController.reset(robotAngleSupplier.getAsDouble(), dRadians);
    }

    @Override
    public void execute() {
        double x, y, omega;

        // cancel setpoint rotation if turning with joystick
        if (useSetpointRotMode && Math.abs(omegaSupp.getAsDouble()) >= 0.1)
            useSetpointRotMode = false;
        // if there don't rotate
        if (rotController.atGoal())
            useSetpointRotMode = false;

        omega = omegaLimiter.calculate(omegaSupp.getAsDouble());
        if (useSetpointRotMode)
            omega = rotController.calculate(robotAngleSupplier.getAsDouble());

        speedScale = MathUtil.interpolate(minSpeed, 1.0, turbo.getAsDouble());

        x = txLimiter.calculate(txSupp.getAsDouble()) * speedScale;
        y = tyLimiter.calculate(tySupp.getAsDouble()) * speedScale;
        if (!useSetpointRotMode)
            omega *= speedScale;

        if (isRed && fieldRelative.getAsBoolean()) {
            x *= -1.0;
            y *= -1.0;
        }

        drive.poutDrive(x, y, omega, fieldRelative.getAsBoolean());
    }

    @Override
    public void end(boolean interrupted) {
        useSetpointRotMode = false;
    }

    public CommandBase setSetpoint(SetpointDirection setpointDirection) {
        return new InstantCommand(() -> {
            rotController.setGoal(setpointDirection.getAngle());
            rotController.reset(robotAngleSupplier.getAsDouble(), drive.getCurrentTwist().dtheta);
            useSetpointRotMode = true;
        }).handleInterrupt(() -> useSetpointRotMode = false);
    }

    public enum SetpointDirection {
        FORWARD(0.0),
        LEFT(Math.PI / 2.0),
        BACKWARD(Math.PI),
        RIGHT(-Math.PI / 2.0);

        private final double angle;
        SetpointDirection(double angle) {
            this.angle = angle;
        }

        public double getAngle() {
            if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
                return MathUtil.inputModulus(angle + Math.PI, -Math.PI, Math.PI);
            return angle;
        }
    }

    private static double signedSquare(double value) {
        return Math.signum(value) * (value * value);
    }
}
