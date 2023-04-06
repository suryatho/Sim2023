// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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

    public static final double ROBOT_WIDTH = 0.7;
    public static final double ROBOT_LENGTH = 0.7;

    public static class DriveConstants {
        public static final double trackwidth = Units.inchesToMeters(26.0);
        public static final double wheelbase = Units.inchesToMeters(26.0);


        public static final double wheelRadius = 0.10033 / 2.0;
        public static final double driveReduction = (14.0 / 50.0) * (28.0 / 16.0) * (15.0 / 45.0);
        private static final double neoFreeSpeedRPM = 5676.0;

        public static final double maxLinearSpeed = neoFreeSpeedRPM / 60.0 *
                driveReduction * wheelRadius * 2.0 * Math.PI;
        public static final double maxAngularSpeed = maxLinearSpeed /
                Math.hypot(trackwidth / 2.0, wheelbase / 2.0);
    }
}
