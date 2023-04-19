// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.arm.StowArm;
import frc.robot.commands.drive.DriveCommand;
import frc.robot.commands.drive.DriveToNode;
import frc.robot.commands.drive.DriveTrajectory;
import frc.robot.subsystems.NodeSelector;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.ArmJointIO;
import frc.robot.subsystems.arm.ArmJointIOSim;
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
    private Arm arm;
    private NodeSelector nodeSelector;

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
                arm = new Arm(
                        new ArmJointIOSim(),
                        new ArmJointIOSim());
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

        if (arm == null)
            arm = new Arm(
                    new ArmJointIO() {},
                    new ArmJointIO() {});

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
        if (Constants.useController) {
            StormXboxController controller = new StormXboxController(0);

            if (Constants.useDriverController) {
                DriveCommand driveCommand = new DriveCommand(
                        drive,
                        () -> -controller.getLeftY(),
                        () -> -controller.getLeftX(),
                        () -> -controller.getRightX(),
                        () -> true,
                        controller::getRightTriggerAxis
                );

                controller.y().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.FORWARD));
                controller.x().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.LEFT));
                controller.a().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.BACKWARD));
                controller.b().onTrue(driveCommand.setSetpoint(DriveCommand.SetpointDirection.RIGHT));
                // schedule drive command
                drive.setDefaultCommand(driveCommand);

                // node selector ui
                controller.povUp().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.DOWN));
                controller.povDown().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.UP));
                controller.povRight().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.RIGHT));
                controller.povLeft().onTrue(nodeSelector.moveSelectedCommand(NodeSelector.Direction.LEFT));

                // drive to node
                controller.leftTrigger().whileTrue(Commands.repeatingSequence(
                        Commands.deadline(
                                new WaitUntilCommand(nodeSelector::isNodeColChanged),
                                new DriveToNode(drive, nodeSelector::getSelectedNode)
                        )));

                // reset pose
                controller.start().onTrue(new InstantCommand(() -> drive.resetPose(new Pose2d())));

                controller.leftBumper().onTrue(new StowArm(arm));
            }

            if (Constants.useArmController) {
                arm.setDefaultCommand(
                        new RunCommand(() -> arm.poutDriveArm(controller.getRightX(), -controller.getLeftY()))
                );
            }
        }
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        PathPlannerTrajectory trajectory = PathPlanner.loadPath("New Path", new PathConstraints(6, 6));
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red)
            drive.resetPose(FlipUtil.apply(trajectory.getInitialHolonomicPose()));
        else
            drive.resetPose(trajectory.getInitialHolonomicPose());
        return new DriveTrajectory(drive, trajectory, true);
    }
}
