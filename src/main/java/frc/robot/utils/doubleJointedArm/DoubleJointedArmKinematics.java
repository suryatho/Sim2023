package frc.robot.utils.doubleJointedArm;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import static java.lang.Math.*;

public class DoubleJointedArmKinematics {

    private final double a1Length, a2Length;
    private double alpha = Math.PI, beta = -Math.PI;

    private Pose2d m_endPose;

    /**
     * kinematics for generating DoubleJointedArmSpeeds from end speeds
     * @param a1Length length of first arm
     * @param a2Length length of second arm
     * @param alpha angle in radians of first joint. 0 parallel to ground PI is perpendicular
     * @param beta angle in radians of second joint. When arm is straight up angles should be j1: PI, j2: 0
     */
    public DoubleJointedArmKinematics(double a1Length, double a2Length, double alpha, double beta) {
        this.a1Length = a1Length;
        this.a2Length = a2Length;
        updateAngles(alpha, beta);
    }

    /** must be called periodically for other methods to work */
    public void updateAngles(double alpha, double beta) {
        this.alpha = alpha;
        this.beta = beta;
        // recalculate end pose
        calculateEndPose();
    }

    /** called internally */
    private void calculateEndPose() {
        // this is the rotation of the end
        Rotation2d sumAngles = new Rotation2d(alpha + beta);
        // add up x and y components of both joints to get end pose
        double x = a1Length * cos(alpha) + a2Length * sumAngles.getCos();
        double y = a1Length * sin(alpha) + a2Length * sumAngles.getSin();
        m_endPose = new Pose2d(x, y, sumAngles);
    }

    public Pose2d getEndPose() {
        return m_endPose;
    }

    /** arm speeds (radians per second) from end speed in m/s */
    public DoubleJointedArmSpeeds fromGripperSpeeds(double vx, double vy) {
        return new DoubleJointedArmSpeeds(alphaSpeed(vx, vy), betaSpeed(vx, vy));
    }

    private double alphaSpeed(double vx, double vy) {;
        double alphaNum = vx * (a2Length * cos(alpha + beta)) + vy * (a1Length * sin(alpha + beta));
        double den = a1Length * a2Length * sin(beta);
        // TODO - handle singularity when den --> 0
        return alphaNum / den;
    }

    private double betaSpeed(double vx, double vy) {
        double betaNum = vx * (-a1Length * cos(alpha) - a2Length * cos(alpha + beta)) +
                vy * (-a1Length * sin(alpha) - a2Length * sin(alpha +  beta));
        double den = a1Length * a2Length * sin(beta);
        return betaNum / den;
    }
}
