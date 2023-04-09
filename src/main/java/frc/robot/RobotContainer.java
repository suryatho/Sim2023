// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToPose;
import frc.robot.commands.drive.TrajectoryFollowingCommand;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.drive.SwerveModuleIO;
import frc.robot.subsystems.drive.SwerveModuleIOSim;
import frc.robot.utils.FlipUtil;
import frc.robot.utils.StormXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    private SwerveDrive drive;
    private NodeSelector nodeSelector;

    private final StormXboxController driver = new StormXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch (Constants.currentMode) {
            // Real robot, instantiate hardware IO implementations
            case REAL:
                break;

            // Sim robot, instantiate physics sim IO implementations
            case SIM:
                drive = new SwerveDrive(
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim(),
                        new SwerveModuleIOSim());
                break;

            // Replayed robot, disable IO implementations
            default:
                break;
        }

        if (drive == null)
            drive = new SwerveDrive(
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {},
                    new SwerveModuleIO() {}
            );

        nodeSelector = new NodeSelector();

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {@link GenericHID} or one of its subclasses
     * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
     * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        DriveCommand driveCommand = new DriveCommand(
                drive,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(),
                () -> !driver.getHID().getRightBumper(),
                driver::getRightTriggerAxis
        );

        driver.y().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.FORWARD));
        driver.x().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.LEFT));
        driver.a().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.BACKWARD));
        driver.b().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.RIGHT));

        drive.setDefaultCommand(driveCommand);

        driver.povUp().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.DOWN));
        driver.povDown().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.UP));
        driver.povRight().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.RIGHT));
        driver.povLeft().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.LEFT));

        driver.leftTrigger().whileTrue(new DriveToPose(drive, () -> nodeSelector.getSelectedNode().scoringPosition));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", new PathConstraints(4, 3));
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
            drive.resetOdometry(FlipUtil.apply(trajectory.getInitialHolonomicPose()));
        else
            drive.resetOdometry(trajectory.getInitialHolonomicPose());
        return new TrajectoryFollowingCommand(drive, trajectory, true);
    }
}
