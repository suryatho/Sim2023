package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.doubleJointedArm.DoubleJointedArmKinematics;
import frc.robot.utils.doubleJointedArm.DoubleJointedArmSpeeds;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ArmConstants.*;

public class Arm extends SubsystemBase {

    private final ArmJointIO shoulder;
    private final ArmJointIO elbow;
    private final ArmJointIOInputsAutoLogged shoulderInputs = new ArmJointIOInputsAutoLogged();
    private final ArmJointIOInputsAutoLogged elbowInputs = new ArmJointIOInputsAutoLogged();

    private final DoubleJointedArmKinematics kinematics;
    private DoubleJointedArmSpeeds speeds = new DoubleJointedArmSpeeds();

    private ArmVisualizer armVisualizer;

    public Arm(ArmJointIO shoulder, ArmJointIO elbow) {
        this.shoulder = shoulder;
        this.elbow = elbow;

        shoulder.setConfig(shoulderConfig);
        elbow.setConfig(elbowConfig);

        // angles aren't true
        kinematics = new DoubleJointedArmKinematics(a1Length, a2Length, maxAlphaAngle, minBetaAngle);

        armVisualizer = new ArmVisualizer();
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled())
            speeds = new DoubleJointedArmSpeeds();

        shoulder.updateInputs(shoulderInputs);
        Logger.getInstance().processInputs("Arm/Shoulder", shoulderInputs);
        elbow.updateInputs(elbowInputs);
        Logger.getInstance().processInputs("Arm/Elbow", elbowInputs);

        kinematics.updateAngles(shoulderInputs.angleRadians, elbowInputs.angleRadians);

        shoulder.setSpeed(speeds.alphaRadiansPerSecond);
        elbow.setSpeed(speeds.betaRadiansPerSecond);

        armVisualizer.updateAngles(shoulderInputs.angleRadians, elbowInputs.angleRadians);
    }

    public void moveEnd(double vx, double vy) {
        speeds = kinematics.fromGripperSpeeds(vx, vy);
        speeds.desaturate(shoulderMaxSpeed, elbowMaxSpeed);
    }

    public void poutMoveEnd(double vx, double vy) {
        vx = MathUtil.clamp(vx, -1.0, 1.0);
        vy = MathUtil.clamp(vy, -1.0, 1.0);
        moveEnd(vx, vy);
    }
}
