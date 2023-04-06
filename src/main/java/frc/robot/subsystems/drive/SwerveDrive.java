package frc.robot.subsystems.drive;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.DriveConstants.*;

public class SwerveDrive extends SubsystemBase {

    private final SwerveModule[] modules = new SwerveModule[4];
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Pose2d odometryPose = new Pose2d();
    private double[] prevWheelDistances = {0.0, 0.0, 0.0, 0.0};

    private SwerveModuleState[] setpointStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    private PPHolonomicDriveController driveController = new PPHolonomicDriveController(
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0),
            new PIDController(3.0, 0.0, 0.0)
    );

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
        Twist2d twist = kinematics.toTwist2d(deltas);
        odometryPose = odometryPose.exp(twist);

        Logger.getInstance().recordOutput("Odometry/Robot", getOdometryPose());
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
            drive(ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(vx, vy, omega), getOdometryPose().getRotation()));
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

    public Pose2d getOdometryPose() {
        return odometryPose;
    }

    public void resetOdometry(Pose2d pose) {
        odometryPose = pose;
    }

    public PPHolonomicDriveController getDriveController() {
        return driveController;
    }
}