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
import frc.robot.utils.poseEstimator.PoseEstimator2;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Twist2d currentTwist = new Twist2d();
    private double[] prevWheelDistances = {0.0, 0.0, 0.0, 0.0};

    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private final PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDController(3.0, 0.0, 0.0),

            new PIDController(3.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0)
    );

    private final PoseEstimator2 poseEstimator;

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

        poseEstimator = new PoseEstimator2(VecBuilder.fill(0.001, 0.001, 0.001));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
            return;
        }

        for (var module : modules)
            module.periodic();

        for (int i = 0; i < modules.length; i++) {
            setpointStates[i] = SwerveModuleState.optimize(setpointStates[i], modules[i].getSteerAngle());
            modules[i].setModuleState(setpointStates[i]);
        }
        Logger.getInstance().recordOutput("SwerveStates/Setpoints", setpointStates);

        SwerveModuleState[] currentStates = new SwerveModuleState[modules.length];
        for (int i = 0 ; i < modules.length; i++) {
            currentStates[i] = modules[i].getState();
        }
        Logger.getInstance().recordOutput("SwerveStates/measured", currentStates);


        SwerveModulePosition[] deltas = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            deltas[i] = new SwerveModulePosition(
                    modules[i].getDriveDistanceMeters() - prevWheelDistances[i], modules[i].getSteerAngle());
            prevWheelDistances[i] = modules[i].getDriveDistanceMeters();
        }
        currentTwist = kinematics.toTwist2d(deltas);
        poseEstimator.addDriveTwist(Timer.getFPGATimestamp(), currentTwist);

        Logger.getInstance().recordOutput("Pose/Robot", getPose());
    }

    public void drive(SwerveModuleState[] states) {
        if (states.length != modules.length) {
            DriverStation.reportWarning("Can't drive with states of length not equal to num of modules", true);
            return;
        }
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

    public Translation2d[] getModuleTranslations() {
        final double halfTrackwidth = trackwidth / 2.0;
        final double halfWheelbase = wheelbase / 2.0;
        return new Translation2d[] {new Translation2d(halfWheelbase, halfTrackwidth), new Translation2d(halfWheelbase, -halfTrackwidth),
                new Translation2d(-halfWheelbase, halfTrackwidth), new Translation2d(-halfWheelbase, -halfTrackwidth)};
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPose();
    }

    public Twist2d getCurrentTwist() {
        return currentTwist;
    }
    public Twist2d getCurrentVel() {
        return new Twist2d(currentTwist.dx / 0.02, currentTwist.dy / 0.02, currentTwist.dtheta / 0.02);
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPose(pose);
    }

    public PPHolonomicDriveController getDriveController() {
        return driveController;
    }
}