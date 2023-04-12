package frc.robot.commands.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.utils.FlipUtil;
import org.littletonrobotics.junction.Logger;

import static com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

public class TrajectoryFollowingCommand extends CommandBase {

    private final SwerveDrive drive;
    private PathPlannerTrajectory trajectory = new PathPlannerTrajectory();
    private boolean useFlipForAlliance = false;

    private Timer timer = new Timer();

    public TrajectoryFollowingCommand(SwerveDrive drive, PathPlannerTrajectory trajectory, boolean useFlipForAlliance) {
        this.drive = drive;
        this.trajectory = trajectory;
        this.useFlipForAlliance = useFlipForAlliance;

        addRequirements(drive);
    }

    public TrajectoryFollowingCommand(SwerveDrive drive) {
        this.drive = drive;
    }

    protected void setTrajectory(PathPlannerTrajectory trajectory) {
        this.trajectory = trajectory;
    }

    protected void setUseFlipForAlliance(boolean useFlipForAlliance) {
        this.useFlipForAlliance = useFlipForAlliance;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        if (useFlipForAlliance)
            trajectory = FlipUtil.apply(trajectory);

        Logger.getInstance().recordOutput("TrajectoryFollowing/SetpointTrajectory",
                trajectory.getStates().stream().map(s -> {
                            PathPlannerState state = (PathPlannerState) s;
                            return new Pose2d(state.poseMeters.getTranslation(), state.holonomicRotation);
                            
                        }).toArray(Pose2d[]::new));
    }

    @Override
    public void execute() {
        PathPlannerState setpointState = (PathPlannerState) trajectory.sample(timer.get());
        Pose2d setpoint = new Pose2d(setpointState.poseMeters.getTranslation(), setpointState.holonomicRotation);
        Logger.getInstance().recordOutput("TrajectoryFollowing/Setpoint", setpoint);

        Pose2d currentPose = drive.getPose();
        Logger.getInstance().recordOutput("TrajectoryFollowing/Robot", currentPose);
        ChassisSpeeds speeds = drive.getDriveController().calculate(currentPose, setpointState);
        System.out.println(speeds);
        drive.drive(speeds);
    }

    @Override
    public boolean isFinished() {
        return (timer.get() >= trajectory.getTotalTimeSeconds() + 1.0) ||
                (timer.get() >= trajectory.getTotalTimeSeconds() && drive.getDriveController().atReference());
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Trajectory Following Command ended after " + timer.get() + " seconds");
        timer.stop();
    }
}