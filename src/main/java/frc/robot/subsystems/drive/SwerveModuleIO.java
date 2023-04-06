package frc.robot.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO {
    @AutoLog
    class SwerveModuleIOInputs {
        public double driveDistanceMeters = 0.0;
        public double driveVelMetersPerSec = 0.0;
        public double steerPositionRad = 0.0;
        public double steerVelRadPerSec = 0.0;
    }

    default void updateInputs(SwerveModuleIOInputs inputs) {}
    default void runDrive(double volts) {}
    default void runSteer(double volts) {}
}
