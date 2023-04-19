// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.subsystems.arm.ArmJointConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final Mode currentMode = Mode.SIM;
    public static final double loopPeriodSecs = 0.02;

    public static enum Mode {
        /**
         * Running on a real robot.
         */
        REAL,

        /**
         * Running a physics simulator.
         */
        SIM,

        /**
         * Replaying from a log file.
         */
        REPLAY
    }

    public static final boolean useController = true;
    public static final boolean useDriverController = true && useController;
    public static final boolean useArmController = false && useController;

    public static final double bumperThickness = Units.inchesToMeters(4.0);
    public static final double robotWidth = Units.inchesToMeters(26.0);
    public static final double robotLength = Units.inchesToMeters(26.0);
    public static final double neoFreeSpeedRPM = 5676.0;

    public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    public static class DriveConstants {
        public static final double trackwidth = 0.65;
        public static final double wheelbase = 0.65;
        public static final double turnRadius = Math.hypot(trackwidth / 2.0, wheelbase / 2.0);

        public static final double wheelRadius = 0.10033 / 2.0;
        public static final double driveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        private static final double neoFreeSpeedRPM = 5676.0;
        public static final double maxLinearSpeed = neoFreeSpeedRPM / 60.0 *
                driveReduction * wheelRadius * 2.0 * Math.PI;
        public static final double maxAngularSpeed = maxLinearSpeed /
                Math.hypot(trackwidth / 2.0, wheelbase / 2.0);
    }

    public static class ArmConstants {
        public static final double a1Length = 0.902;
        public static final double a2Length = 0.889;

        // B.S.
        public static final double a1Kg = 6.97;
        public static final double a2Kg = 6.7;

        public static final double shoulderOffsetRadians = 3.799277345;
        public static final double elbowOffsetRadians = 4.878652344;

        public static final double shoulderGearRatio = 205.0;
        public static final double elbowGearRatio = 205.0;

        public static final double shoulderMaxSpeed = (90.0 / 360.0) * 2 * Math.PI;
        public static final double elbowMaxSpeed = shoulderMaxSpeed * 0.9;

//        public static final double minAlphaAngle = -Math.PI;
//        public static final double maxAlphaAngle = Math.PI;
//        public static final double minBetaAngle = -Math.PI;
//        public static final double maxBetaAngle = Math.PI;
        public static final double minAlphaAngle = Math.PI / 6.0;
        public static final double maxAlphaAngle = (120.0 / 360.0) * 2 * Math.PI;
        public static final double maxBetaAngle = -Math.PI / 30.0;
        public static final double minBetaAngle = (-165.0 / 360.0) * 2 * Math.PI;

        public static final ArmJointConfig shoulderConfig = new ArmJointConfig(
                shoulderOffsetRadians,
                shoulderMaxSpeed,
                a1Length,
                minAlphaAngle,
                maxAlphaAngle,
                a1Kg,
                shoulderGearRatio);

        public static final ArmJointConfig elbowConfig = new ArmJointConfig(
                elbowOffsetRadians,
                elbowMaxSpeed,
                a2Length,
                minBetaAngle,
                maxBetaAngle,
                a2Kg,
                elbowGearRatio);

        public static final Translation3d armOrigin3d = new Translation3d(Units.inchesToMeters(5.0), 0.0, Units.inchesToMeters(9.0));
        public static final Translation2d armOrigin2dArmSpace = new Translation2d(Units.inchesToMeters(5.0), Units.inchesToMeters(9.0));

        public static final Translation2d stowedPosition = new Translation2d(0.13, 0.25);
    }
}
