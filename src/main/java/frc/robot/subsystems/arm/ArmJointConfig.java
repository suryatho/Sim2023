package frc.robot.subsystems.arm;

public class ArmJointConfig {
    public final double offsetRadians;
    public final double maxSpeed;
    public final double length;
    public final double minAngle, maxAngle;

    public ArmJointConfig(double offsetRadians, double maxSpeed, double length, double minAngle, double maxAngle) {
        this.offsetRadians = offsetRadians;
        this.length = length;
        this.maxSpeed = maxSpeed;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
    }

    public ArmJointConfig() {
        this(0.0, 0.0, 0.0, 0.0, 0.0);
    }
}
