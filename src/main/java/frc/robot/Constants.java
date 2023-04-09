// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

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

    public static final double bumperThickness = Units.inchesToMeters(5.0);
    public static final double robotWidth = Units.inchesToMeters(26.0);
    public static final double robotLength = Units.inchesToMeters(26.0);
    public static final double neoFreeSpeedRPM = 5676.0;

    public static final ShuffleboardTab driverTab = Shuffleboard.getTab("Driver");

    public static class DriveConstants {
        public static final double trackwidth = Units.inchesToMeters(24.0);
        public static final double wheelbase = Units.inchesToMeters(24.0);

        public static final double wheelRadius = 0.10033 / 2.0;
        public static final double driveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        private static final double neoFreeSpeedRPM = 5676.0;

        public static final double maxLinearSpeed = neoFreeSpeedRPM / 60.0 *
                driveReduction * wheelRadius * 2.0 * Math.PI;
        public static final double maxAngularSpeed = maxLinearSpeed /
                Math.hypot(trackwidth / 2.0, wheelbase / 2.0);
    }

    public static class ArmConstants {
        public static final double gearRatio = 1.0 / 200.0;
        public static final double maxJointSpeedRadiansPerSecond = (neoFreeSpeedRPM / 60.0) * (gearRatio) *
                (2 * Math.PI);

        public static final double minAlphaAngle = Math.PI / 6.0;
        public static final double maxAlphaAngle = (110.0 / 360.0) * 2 * Math.PI;
        public static final double minBetaAngle = -Math.PI / 30.0;
        public static final double maxBetaAngle = (-150.0 / 360.0) * 2 * Math.PI;
    }
}
