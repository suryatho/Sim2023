package frc.robot.subsystems.arm;

import org.littletonrobotics.junction.AutoLog;

public interface ArmJointIO {
    @AutoLog
    class ArmJointIOInputs {
        double angleRadians = 0.0;
        double angleRadiansPerSecond = 0.0;
    }

    default void updateInputs(ArmJointIOInputs inputs) {}

    /** set the speed of the join in radians per second */
    default void setSpeed(double speed) {}

    default void setConfig(ArmJointConfig config) {}
}
