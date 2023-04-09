package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.subsystems.NodeSelector.Node;
import static frc.robot.subsystems.NodeSelector.Node.*;

public final class FieldConstants {

    public static final double FIELD_LENGTH = Units.feetToMeters(54.0) + Units.inchesToMeters(3.25);
    public static final double FIELD_WIDTH = Units.feetToMeters(26) + Units.inchesToMeters(3.5);

    public static class Grids {
        public static final Node[][] blueAllianceGrid = new Node[9][3];
        public static final Node[][] redAllianceGrid = new Node[9][3];

        private static final double distBetweenNodes = Units.inchesToMeters(22.0);
        private static final double distToFirstNodeY = Units.inchesToMeters(20.0);
        private static final double[] nodeXs = {Units.inchesToMeters(14.32),
                Units.inchesToMeters(31.35), Units.inchesToMeters(47.0)};
        private static final double[] nodeZs = {Units.inchesToMeters(46.0),
                Units.inchesToMeters(34.0), Units.inchesToMeters(12.0)};

        private static final Height[] heights = {Height.HIGH, Height.MID, Height.HYBRID};

        static {
            double alignedX = Units.inchesToMeters(54.0) +
                    (Constants.robotLength + Constants.bumperThickness) / 2.0;
            for (int col = 0; col <= 8; col++) {
                // get type of node
                boolean isCube = col == 1 || col == 4 || col == 7;
                Type type = isCube? Type.CUBE : Type.CONE;

                double nodeY = distToFirstNodeY + (distBetweenNodes * col);

                for (int row = 0; row <= 2; row++) {
                    // all nodes in row two are hybrid
                    if (row == 2)
                        type = Type.HYBRID;

                    // get node height enum
                    Height height = heights[row];

                    // get X position of node
                    double nodeX = nodeXs[row];
                    // get height of node subtract if cube
                    double nodeZ = nodeZs[row] - (isCube? 5.0 : 0.0);
                    // make position objects
                    Translation3d nodePosition = new Translation3d(nodeX, nodeY, nodeZ);
                    Pose2d scoringPosition = new Pose2d(alignedX, nodeY, new Rotation2d(Math.PI));

                    Node blue = new Node(row, col, nodePosition, scoringPosition,
                            DriverStation.Alliance.Blue, height, type);
                    Node red = Node.getRedAbsolute(blue);
                    blueAllianceGrid[col][row] = blue;
                    redAllianceGrid[col][row] = red;
                }
            }
        }
    }
}
