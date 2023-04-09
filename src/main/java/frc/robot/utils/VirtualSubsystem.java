package frc.robot.utils;

import java.util.ArrayList;
import java.util.List;

public class VirtualSubsystem {

    private static final List<VirtualSubsystem> subsystemList = new ArrayList<>();

    public VirtualSubsystem() {
        subsystemList.add(this);
    }

    public void periodic() {}

    public static void run() {
        for (var virtualSubsystem : subsystemList)
            virtualSubsystem.periodic();
    }
}
