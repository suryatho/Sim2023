package frc.robot.subsystems.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.PoseEstimator;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Twist2d currentTwist = new Twist2d();
    private final double[] prevWheelDistances = {0.0, 0.0, 0.0, 0.0};
    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDController(maxLinearSpeed / 2.0, 0.0, 0.0),
            new PIDController(maxLinearSpeed / 2.0, 0.0, 0.0),
            new PIDController(maxAngularSpeed / Math.PI, 0.0, 0.0)
    );

    private Pose2d previousPose = new Pose2d();
    private ChassisSpeeds currentVelocity = new ChassisSpeeds();
    private final PoseEstimator poseEstimator;
    // used for calculating velocity
    private double loopTime = Constants.loopPeriodSecs;

    public static SwerveModuleState[] XStates =
            Arrays.stream(getModuleTranslations())
                .map(translation -> new SwerveModuleState(0.0, translation.getAngle()))
                .toArray(SwerveModuleState[]::new);

    public SwerveDrive(
            SwerveModuleIO fl,
            SwerveModuleIO fr,
            SwerveModuleIO bl,
            SwerveModuleIO br) {

        modules[0] = new SwerveModule(fl, 0);
        modules[1] = new SwerveModule(fr, 1);
        modules[2] = new SwerveModule(bl, 2);
        modules[3] = new SwerveModule(br, 3);

        for (int i = 0; i < modules.length; i++) {
            prevWheelDistances[i] = modules[i].getDriveDistanceMeters();
        }

        driveController.setTolerance(new Pose2d(0.05, 0.05, new Rotation2d((0.5 / 360.0) * 2 * Math.PI)));

        poseEstimator = new PoseEstimator(VecBuilder.fill(0.01, 0.01, 0.01));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            return;
        }

        // must be called on all modules periodically
        for (var module : modules)
            module.periodic();

        // run the modules
        for (int i = 0; i < modules.length; i++) {
            setpointStates[i] = SwerveModuleState.optimize(setpointStates[i], modules[i].getSteerAngle());
            modules[i].setModuleState(setpointStates[i]);
        }
        Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);

        // log current measured states
        SwerveModuleState[] currentStates = new SwerveModuleState[modules.length];
        for (int i = 0 ; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/measured", currentStates);

        // calculate the delta and twist from previous timestep to add to pose estimator
        SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            deltas[i] = new SwerveModulePosition(
                    modules[i].getDriveDistanceMeters() - prevWheelDistances[i], modules[i].getSteerAngle());
            prevWheelDistances[i] = modules[i].getDriveDistanceMeters();
        }

        // get previous pose
        previousPose = getPose();

        currentTwist = kinematics.toTwist2d(deltas);
        poseEstimator.addDriveTwist(Timer.getFPGATimestamp(), currentTwist);

        // get difference in time
        loopTime = Timer.getFPGATimestamp() - loopTime;
        if (loopTime >= Constants.loopPeriodSecs * 20.0)
            loopTime = Constants.loopPeriodSecs;

        // get currentVelocity
        Pose2d currentPose = getPose();
        currentVelocity = new ChassisSpeeds(
                (currentPose.getX() - previousPose.getX()) / loopTime,
                (currentPose.getY() - previousPose.getY()) / loopTime,
                (currentPose.getRotation().getRadians() - previousPose.getRotation().getRadians()) / loopTime
        );

        Logger.getInstance().recordOutput("Pose/Robot", currentPose) ;
        Logger.getInstance().recordOutput("Pose/RobotVel", new double[] {
                getCurrentVel().vxMetersPerSecond,
                getCurrentVel().vyMetersPerSecond,
                getCurrentVel().omegaRadiansPerSecond});
    }

    public void drive(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            DriverStation.reportWarning("Can't drive with states of length not equal to num of modules", true);
            return;
        }
        // the max linear speed is how fast the drive motor can go
        SwerveDriveKinematics.desaturateWheelSpeeds(states, maxLinearSpeed);
        setpointStates = states;
    }

    public void drive(ChassisSpeeds speeds) {
        drive(kinematics.toSwerveModuleStates(speeds));
    }

    public void drive(double vx, double vy, double omega, boolean fieldRelative) {
        if (fieldRelative) {
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, omega), getPose().getRotation()));
            return;
        }
        drive(new ChassisSpeeds(vx, vy, omega));
    }
    
    public void poutDrive(double vx, double vy, double omega, boolean fieldRelative) {
        // clamp so we don't go above max speed
        double clampX = MathUtil.clamp(vx, -1.0, 1.0);
        double clampY = MathUtil.clamp(vy, -1.0, 1.0);
        double clampOmega = MathUtil.clamp(omega, -1.0, 1.0);

        drive(clampX * maxLinearSpeed, clampY * maxLinearSpeed, clampOmega * maxAngularSpeed, fieldRelative);
    }

    public void stop() {
        drive(new ChassisSpeeds(0, 0, 0));
    }

    public void stopWithX() {
        drive(XStates);
    }

    public static Translation2d[] getModuleTranslations() {
        final double halfTrackwidth = trackwidth / 2.0;
        final double halfWheelbase = wheelbase / 2.0;
        return new Translation2d[] {
                new Translation2d(halfWheelbase, halfTrackwidth),
                new Translation2d(halfWheelbase, -halfTrackwidth),
                new Translation2d(-halfWheelbase, halfTrackwidth),
                new Translation2d(-halfWheelbase, -halfTrackwidth)};
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPose();
    }

    public Twist2d getCurrentTwist() {
        return currentTwist;
    }

    public ChassisSpeeds getCurrentVel() {
        return currentVelocity;
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public PPHolonomicDriveController getDriveController() {
        return driveController;
    }
}