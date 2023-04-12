package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ArmConstants.*;

public class ArmVisualizer {

    private final Mechanism2d mechanism;
    private final MechanismLigament2d shoulderLigament;
    private final MechanismLigament2d elbowLigament;

    public ArmVisualizer() {
        mechanism = new Mechanism2d(2.5, 2, new Color8Bit(Color.kGainsboro));
        MechanismRoot2d mechanismRoot = mechanism.getRoot("Origin", armOrigin2dArmSpace.getX() + 2.5/2.0, armOrigin2dArmSpace.getY());
        shoulderLigament = mechanismRoot.append(
                new MechanismLigament2d("Shoulder", a1Length, Units.radiansToDegrees(maxAlphaAngle)));
        elbowLigament = shoulderLigament.append(
                new MechanismLigament2d("Elbow", a2Length, Units.radiansToDegrees(minBetaAngle)));
    }

    public void updateAngles(double alpha, double beta) {
        shoulderLigament.setAngle(Units.radiansToDegrees(alpha));
        elbowLigament.setAngle(Units.radiansToDegrees(beta));
        Logger.getInstance().recordOutput("Arm/Measured", mechanism);
    }
}
