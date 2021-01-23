package frc.robot.commands.drivecommands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class BackupCommand extends CommandBase {

    /**
     * Backup a set distance using ramsete command
     * 
     * @param driveSubsystem The subsystem to be used
     * @param distance       The distance to backup (meters)
     */
    public BackupCommand(DriveSubsystem driveSubsystem, double distance) {
        Trajectory backup = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(),
                new Pose2d(new Translation2d(distance, 0), new Rotation2d()), DriveConstants.kTrajectoryConfig);
        new TrajectoryFollowCommand(driveSubsystem, backup.relativeTo(driveSubsystem.getPose()));
    }
}