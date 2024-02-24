package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.utils.NetworkTableUtils;

import java.util.List;

public class PathCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;

    private PathPlannerPath path;

    private Pose2d endPose;


    public PathCommand(SwerveSubsystem swerveSubsystem, Pose2d endPose) {
        this.swerveSubsystem = swerveSubsystem;
        this.endPose = endPose;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        Pose2d startPos = new Pose2d(
                swerveSubsystem.getPose().getTranslation(),
                new Rotation2d()
        );

        Pose2d endPos = new Pose2d(endPose.getTranslation(), new Rotation2d());

        List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(startPos, endPos);
        path = new PathPlannerPath(
                bezierPoints,
                new PathConstraints(
                        4.0, 4.0,
                        2 * Math.PI, 2.5 * Math.PI
                ),
                new GoalEndState(0.0, endPos.getRotation())
        );

        path.preventFlipping = true;

        AutoBuilder.followPath(path).schedule();

    }

    @Override
    public void end(boolean i) {
        swerveSubsystem.drive(0, 0, 0, false, false);
    }
}
