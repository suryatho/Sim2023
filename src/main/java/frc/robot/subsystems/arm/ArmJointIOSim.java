package frc.robot.subsystems.arm;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmJointIOSim implements ArmJointIO {
    private final SingleJointedArmSim jointMotor;
    private final ArmJointConfig config;

    public ArmJointIOSim(ArmJointConfig config) {
        setConfig(config);
        jointMotor = new SingleJointedArmSim(DCMotor.getNEO(1), 20.0,
                config.length, config.minAngle, config.maxAngle, false);
    }

    @Override
    public void updateInputs(ArmJointIOInputs inputs) {
    }

    @Override
    public void setConfig(ArmJointConfig config) {
        this.config = config;
    }

    @Override
    public void setSpeed(double speed) {
        ArmJointIO.super.setSpeed(speed);
    }
}
