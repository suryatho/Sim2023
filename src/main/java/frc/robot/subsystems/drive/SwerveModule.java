package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class SwerveModule {
    private final SwerveModuleIO io;
    private final int id;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.001, 0.15);
    private final PIDController driveVelController = new PIDController(0.9, 0.0, 0.0, Constants.loopPeriodSecs);
    private final PIDController steerController = new PIDController(23.0, 0.0, 0.0, Constants.loopPeriodSecs);

    public SwerveModule(SwerveModuleIO io, int id) {
        this.io = io;
        this.id = id;

        steerController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /** must be called on all modules periodically */
    public void periodic() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive/SwerveModule" + id, inputs);
    }

    public void setModuleState(SwerveModuleState state) {
        io.runSteer(
                steerController.calculate(getSteerAngle().getRadians(), state.angle.getRadians()));
        io.runDrive(driveFF.calculate(state.speedMetersPerSecond) + driveVelController.calculate(inputs.driveVelMetersPerSec, state.speedMetersPerSecond));
    }

    public void stop() {
        io.runSteer(0.0);
        io.runDrive(0.0);
    }

    public double getDriveDistanceMeters() {
        return inputs.driveDistanceMeters;
    }

    public double getDriveVelMetersPerSec() {
        return inputs.driveVelMetersPerSec;
    }

    public Rotation2d getSteerAngle() {
        return new Rotation2d(MathUtil.angleModulus(inputs.steerPositionRad));
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelMetersPerSec(), getSteerAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistanceMeters(), getSteerAngle());
    }
}
