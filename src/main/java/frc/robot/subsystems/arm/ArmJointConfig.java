package frc.robot.subsystems.arm;

public class ArmJointConfig {
    public final double offsetRadians;
    public final double maxSpeed;
    public final double length;
    public final double minAngle, maxAngle;
    public final double massKg;
    public final double gearRatio;

    public ArmJointConfig(double offsetRadians, double maxSpeed, double length, double minAngle, double maxAngle, double massKg, double gearRatio) {
        this.offsetRadians = offsetRadians;
        this.length = length;
        this.maxSpeed = maxSpeed;
        this.minAngle = minAngle;
        this.maxAngle = maxAngle;
        this.massKg = massKg;
        this.gearRatio = gearRatio;
    }

    public ArmJointConfig() {
        this(0.0, 0.1, 1.0, 0.0, 1.0, 1.0, 1.0);
    }
}
