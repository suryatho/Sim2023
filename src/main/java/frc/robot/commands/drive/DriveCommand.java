package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
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
            new TrapezoidProfile.Constraints(maxRotVel, maxRotAcc));

    private static double minRotVel = Constants.DriveConstants.maxAngularSpeed * 0.3;
    private static double maxRotVel = Constants.DriveConstants.maxAngularSpeed;
    private static double minRotAcc = Constants.DriveConstants.maxAngularSpeed * 0.6;
    private static double maxRotAcc = Constants.DriveConstants.maxAngularSpeed * 2.3;

    private boolean useSetpointRotMode = false;

    private static final double minSpeed = 0.3;
    private double speedScale = minSpeed;

    private SlewRateLimiter txLimiter = new SlewRateLimiter(3.5),
            tyLimiter = new SlewRateLimiter(3.5),
            omegaLimiter = new SlewRateLimiter(3.5);

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
        this.turbo = () -> signedSquare(turbo.getAsDouble());

        this.robotAngleSupplier = () -> drive.getPose().getRotation().getRadians();

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

        rotController.reset(robotAngleSupplier.getAsDouble(), drive.getCurrentVel().omegaRadiansPerSecond);
        System.out.println("DriveCommand initialize()");
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

        // get omega value from either controller or joystick input
        if (useSetpointRotMode) {
            // calculate max vel and acceleration
            ChassisSpeeds currentVel = drive.getCurrentVel();
            double linearVelScale = Math.hypot(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond) / Constants.DriveConstants.maxLinearSpeed;
            linearVelScale = speedToConstraintFunc(linearVelScale);

            double maxVel = MathUtil.interpolate(maxRotVel, minRotVel, linearVelScale);
            double maxAcc = MathUtil.interpolate(maxRotAcc, minRotAcc, linearVelScale);
            rotController.setConstraints(new TrapezoidProfile.Constraints(maxVel, maxAcc));

            omega = rotController.calculate(robotAngleSupplier.getAsDouble());
        } else
            omega = omegaLimiter.calculate(omegaSupp.getAsDouble());


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
            rotController.reset(robotAngleSupplier.getAsDouble(), drive.getCurrentVel().omegaRadiansPerSecond);
            useSetpointRotMode = true;
        });
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
                return MathUtil.angleModulus(angle + Math.PI);
            return angle;
        }
    }

    private static double signedSquare(double value) {
        return Math.signum(value) * (value * value);
    }

    private static double speedToConstraintFunc(double value) {
        value = MathUtil.clamp(value, 0.0, 1.0);
        return Math.sqrt(value);
    }
}
