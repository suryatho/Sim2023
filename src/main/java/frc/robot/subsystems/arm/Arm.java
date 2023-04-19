package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
    private ChassisSpeeds gripperSpeed = new ChassisSpeeds();
    private DoubleJointedArmSpeeds jointSpeeds = new DoubleJointedArmSpeeds();

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
            jointSpeeds = new DoubleJointedArmSpeeds();

        shoulder.updateInputs(shoulderInputs);
        Logger.getInstance().processInputs("Arm/Shoulder", shoulderInputs);
        elbow.updateInputs(elbowInputs);
        Logger.getInstance().processInputs("Arm/Elbow", elbowInputs);

        kinematics.updateAngles(shoulderInputs.angleRadians, elbowInputs.angleRadians);

        shoulder.setSpeed(jointSpeeds.alphaRadiansPerSecond);
        elbow.setSpeed(jointSpeeds.betaRadiansPerSecond);

        armVisualizer.updateAngles(shoulderInputs.angleRadians, elbowInputs.angleRadians);
    }

    public void driveArm(ChassisSpeeds speeds) {
        gripperSpeed = speeds;
        jointSpeeds = kinematics.fromGripperSpeeds(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
        jointSpeeds.desaturate(shoulderMaxSpeed, elbowMaxSpeed);
    }

    public void driveArm(double vx, double vy) {
        driveArm(new ChassisSpeeds(vx, vy, 0));
    }

    public void poutDriveArm(double vx, double vy) {
        vx = MathUtil.clamp(vx, -1.0, 1.0);
        vy = MathUtil.clamp(vy, -1.0, 1.0);
        driveArm(vx, vy);
    }

    public Pose2d getEndPose() {
        return kinematics.getEndPose();
    }

    public ChassisSpeeds getGripperSpeed() {
        return new ChassisSpeeds(gripperSpeed.vxMetersPerSecond, gripperSpeed.vyMetersPerSecond, gripperSpeed.omegaRadiansPerSecond);
    }

    public static Translation2d fromGlobalTranslation(Translation3d translation) {
        return new Translation2d(translation.getX(), translation.getZ());
    }
}
