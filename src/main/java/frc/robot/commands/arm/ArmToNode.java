package frc.robot.commands.arm;

import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.arm.Arm;

import java.util.function.Supplier;

public class ArmToNode extends ArmTrajectory {

    private final Arm arm;
    private final Supplier<NodeSelector.Node> nodeSupplier;

    public ArmToNode(Arm arm, Supplier<NodeSelector.Node> nodeSupplier) {
        super(arm);
        this.arm = arm;
        this.nodeSupplier = nodeSupplier;
    }

    public ArmToNode(Arm arm, NodeSelector.Node node) {
        this(arm, () -> node);
    }

    @Override
    public void initialize() {

        super.initialize();
    }
}
