package frc.robot.utils;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.math.MathUtil.applyDeadband;

public class StormXboxController extends CommandXboxController {

    private static final double deadZone = 0.2;

    /**
     * Construct an instance of a controller.
     *
     * @param port The port index on the Driver Station that the controller is plugged into.
     */
    public StormXboxController(int port) {
        super(port);
    }

    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX(), deadZone);
    }

    @Override
    public double getLeftY() {
        
        return applyDeadband(super.getLeftY(), deadZone);
    }

    @Override
    public double getRightX() {
        return applyDeadband(super.getRightX(), deadZone);
    }

    @Override
    public double getRightY() {
        return applyDeadband(super.getRightY(), deadZone);
    }
}
