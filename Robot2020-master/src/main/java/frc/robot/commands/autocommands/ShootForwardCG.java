package frc.robot.commands.autocommands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.drivecommands.TrajectoryFollowCommand;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ShootForwardCG extends SequentialCommandGroup {

    /**
     * Shoot balls move forward off init
     */
    public ShootForwardCG(DriveSubsystem driveSubsystem, FlywheelSubsystem flywheelSubsystem,
            HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem, CarouselSubsystem carouselSubsystem) {

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), List.of(),
                new Pose2d(1, 0, new Rotation2d()), DriveConstants.kTrajectoryConfig);
        Command moveForward = new TrajectoryFollowCommand(driveSubsystem, trajectory);
        driveSubsystem.resetOdometry(trajectory.getInitialPose());
        addCommands(new ShootCG(flywheelSubsystem, hoodSubsystem, feederSubsystem, carouselSubsystem), moveForward);
    }
}