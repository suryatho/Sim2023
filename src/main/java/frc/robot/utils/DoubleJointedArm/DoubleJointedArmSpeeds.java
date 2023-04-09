package frc.robot.utils.DoubleJointedArm;

import static java.lang.Math.*;

public class DoubleJointedArmSpeeds {

    public double alphaRadiansPerSecond,
            betaRadiansPerSecond;

    public DoubleJointedArmSpeeds(double alphaRadiansPerSecond, double betaRadiansPerSecond) {
        this.alphaRadiansPerSecond = alphaRadiansPerSecond;
        this.betaRadiansPerSecond = betaRadiansPerSecond;
    }

    public DoubleJointedArmSpeeds() {
        this(0, 0);
    }

    public void desaturate(double maxAlphaRadiansPerSeconds,
                           double maxBetaRadiansPerSecond) {
        double div = max(1.0,
                (max(abs(alphaRadiansPerSecond) / maxAlphaRadiansPerSeconds,
                        abs(betaRadiansPerSecond) / maxBetaRadiansPerSecond)));

        scale(1.0 / div);
    }

    public void scale(double scale) {
        alphaRadiansPerSecond *= scale;
        betaRadiansPerSecond *= scale;
    }

    @Override
    public String toString() {
        return "DoubleJointedArmSpeeds{" +
                "alphaRadiansPerSecond=" + alphaRadiansPerSecond +
                ", betaRadiansPerSecond=" + betaRadiansPerSecond +
                '}';
    }
}
