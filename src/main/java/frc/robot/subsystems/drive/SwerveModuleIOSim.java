package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class SwerveModuleIOSim implements SwerveModuleIO {
    private final FlywheelSim drive = new FlywheelSim(DCMotor.getNEO(1), 6.75, 0.025);
    private final FlywheelSim steer = new FlywheelSim(DCMotor.getNEO(1), 150.0 / 7.0, 0.004);

    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        drive.update(Constants.loopPeriodSecs);
        steer.update(Constants.loopPeriodSecs);

        double steerVelRadPerSec = steer.getAngularVelocityRadPerSec();
        inputs.steerPositionRad += MathUtil.inputModulus(steerVelRadPerSec * Constants.loopPeriodSecs, -Math.PI, Math.PI);
        inputs.steerVelRadPerSec = steerVelRadPerSec;

        double driveVelMetersPerSec = (drive.getAngularVelocityRadPerSec() * (2 * Math.PI * Constants.DriveConstants.wheelRadius));
        inputs.driveDistanceMeters += driveVelMetersPerSec * Constants.loopPeriodSecs;
        inputs.driveVelMetersPerSec = driveVelMetersPerSec;
    }

    @Override
    public void runDrive(double volts) {
        drive.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }

    @Override
    public void runSteer(double volts) {
        steer.setInputVoltage(MathUtil.clamp(volts, -12.0, 12.0));
    }
}
