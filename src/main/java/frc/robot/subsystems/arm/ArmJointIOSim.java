package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class ArmJointIOSim implements ArmJointIO {
    private SingleJointedArmSim joint;
    private ArmJointConfig config;

    public ArmJointIOSim() {}

    @Override
    public void updateInputs(ArmJointIOInputs inputs) {
        joint.update(Constants.loopPeriodSecs);

        inputs.angleRadiansPerSecond = joint.getVelocityRadPerSec();
        inputs.angleRadians = MathUtil.clamp(joint.getAngleRads(), config.minAngle, config.maxAngle);
    }

    @Override
    public void setConfig(ArmJointConfig config) {
        this.config = config;
        double moi = SingleJointedArmSim.estimateMOI(config.length, config.massKg);
        joint = new SingleJointedArmSim(DCMotor.getNEO(1), config.gearRatio, moi,
                config.length, config.minAngle, config.maxAngle, false);
    }

    @Override
    public void setSpeed(double speed) {
        double newAngle = joint.getAngleRads() + (speed * Constants.loopPeriodSecs);
        // clamp joint angle
        if (joint.wouldHitLowerLimit(newAngle) || joint.wouldHitUpperLimit(newAngle))
            speed = 0;
        joint.setInputVoltage((speed / config.maxSpeed) * 12.0);
    }
}
