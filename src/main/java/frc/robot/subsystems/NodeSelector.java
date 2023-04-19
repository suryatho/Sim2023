package frc.robot.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.FieldConstants;
import frc.robot.utils.FlipUtil;
import frc.robot.utils.VirtualSubsystem;

import java.util.Map;

public class NodeSelector extends VirtualSubsystem {

    private int selectedRow = 0, selectedCol = 0;
    private int lastSelectedRow = selectedRow, lastSelectedCol = selectedCol;
    private boolean nodeColChanged = false;
    private boolean nodeRowChanged = false;
    private boolean nodeChanged = false;

    private final static String coneColor = Color.kYellow.toHexString();
    private final static String cubeColor = Color.kPurple.toHexString();
    private final static String hybridColor = Color.kLightGray.toHexString();
    private final static String selectedColor = Color.kLightGreen.toHexString();


    public NodeSelector() {
        // make the layout on shuffleboard
        ShuffleboardLayout gridLayout = Constants.driverTab
                .getLayout("Node Selector", BuiltInLayouts.kGrid)
                .withProperties(Map.of(
                        "Label position", "HIDDEN",
                        "Number of columns", 9,
                        "Number of rows", 3))
                .withPosition(0, 0).withSize(4, 2);
        // init all boolean boxes onto layout
        boolean[] cubeColumns = {false, true, false, false, true, false, false, true, false};
        for (int row = 2; row >= 0; row--) {
            final int tempRow = row;
            for (int col = 8; col >= 0; col--) {
                final int tempCol = col;
                String color = cubeColumns[tempCol]? cubeColor : coneColor;
                color = row == 2? hybridColor : color;
                gridLayout
                        .addBoolean("Column " + (8 - col) + " Row " + (2 - row),
                                () -> tempCol == selectedCol && tempRow == selectedRow)
                        .withProperties(Map.of("color when true", selectedColor,
                                "color when false", color))
                        .withPosition(8 - col, 2 - row);
            }
        }
    }

    @Override
    public void periodic() {
        if (lastSelectedCol != selectedCol) {
            lastSelectedCol = selectedCol;
            nodeColChanged = true;
        } else
            nodeColChanged = false;

        if (lastSelectedRow != selectedRow) {
            lastSelectedRow = selectedRow;
            nodeRowChanged = true;
        } else
            nodeRowChanged = false;

        nodeChanged = nodeColChanged || nodeRowChanged;
    }

    public boolean isNodeChanged() {
        return nodeChanged;
    }

    public boolean isNodeColChanged() {
        return nodeColChanged;
    }

    public boolean isNodeRowChanged() {
        return nodeRowChanged;
    }

    public Node getSelectedNode() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return FieldConstants.Grids.redAllianceGrid[8 - selectedCol][selectedRow];
        }
        return FieldConstants.Grids.blueAllianceGrid[selectedCol][selectedRow];
    }

    /** cols and rows start with highest node on right of blue driver station as (0, 0) */
    public void setSelectedColAndRow(int col, int row) {
        int clampedCol = MathUtil.clamp(col, 0, 8);
        int clampedRow = MathUtil.clamp(row, 0, 3);
        selectedCol = clampedCol;
        selectedRow = clampedRow;
    }

    public CommandBase setSelectedColAndRowCommand(int col, int row) {
        return new InstantCommand(() -> setSelectedColAndRow(col, row));
    }

    public void setSelectedCol(int col) {
        setSelectedColAndRow(col, selectedRow);
    }

    public void setSelectedRow(int row) {
        setSelectedColAndRow(selectedCol, row);
    }

    public void moveSelected(Direction direction) {
        switch (direction) {
            case UP:
            case DOWN:
                selectedRow += direction.val;
                selectedRow = MathUtil.clamp(selectedRow, 0, 2);
                break;
            case LEFT:
            case RIGHT:
                selectedCol += direction.val;
                selectedCol = MathUtil.clamp(selectedCol, 0, 8);
                break;
        }
    }

    public CommandBase moveSelectedCommand(Direction direction) {
        return new InstantCommand(() -> moveSelected(direction)).ignoringDisable(true);
    }

    public enum Direction {
        UP(-1),
        DOWN(1),
        LEFT(1),
        RIGHT(-1);

        public final int val;

        Direction(int val) {
            this.val = val;
        }
    }

    public static class Node {
        public final int row, col;
        public final Translation3d nodePosition;
        public final Pose2d scoringPosition;
        public final DriverStation.Alliance alliance;
        public final Height height;
        public final Type type;

        public Node(int row,
                    int col,
                    Translation3d nodePosition,
                    Pose2d scoringPosition,
                    DriverStation.Alliance alliance,
                    Height height,
                    Type type) {
            this.row = row;
            this.col = col;
            this.nodePosition = nodePosition;
            this.scoringPosition = scoringPosition;
            this.alliance = alliance;
            this.height = height;
            this.type = type;
        }

        /** makes the red node directly mirrored from blue node */
        public static Node getRedAbsolute(Node blueNode) {
            if (blueNode.alliance == DriverStation.Alliance.Red)
                return blueNode;

            Translation3d nodePosition = new Translation3d(FlipUtil.apply(blueNode.nodePosition.getX()),
                    blueNode.nodePosition.getY(),
                    blueNode.nodePosition.getZ());
            Pose2d scoringPosition = FlipUtil.apply(blueNode.scoringPosition);

            return new Node(blueNode.row, blueNode.col, nodePosition, scoringPosition,
                    DriverStation.Alliance.Red, blueNode.height, blueNode.type);
        }

        public enum Height {
            HIGH, MID, HYBRID;
        }

        public enum Type {
            CONE, CUBE, HYBRID
        }
    }
}
