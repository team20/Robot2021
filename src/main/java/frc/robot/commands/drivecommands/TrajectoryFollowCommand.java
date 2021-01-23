package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TrajectoryFollowCommand extends RamseteCommand {

    private final DriveSubsystem m_driveSubsystem;

    /**
     * Follow a trajectory, either created through Pathweaver or manually
     * 
     * @param driveSubsystem The subsystem to be used
     * @param trajectory     The trajectory that the ramsete controller seeks to
     *                       follow
     */
    public TrajectoryFollowCommand(DriveSubsystem driveSubsystem, Trajectory trajectory) {
        // This class serves to abstract away creating the RamseteCommand by just taking
        // in a trajectory and handing the rest
        super(trajectory, driveSubsystem::getPose, new RamseteController(), DriveConstants.kDriveKinematics,
                (left, right) -> driveSubsystem.setWheelSpeeds(new DifferentialDriveWheelSpeeds(left, right)),
                driveSubsystem);
        m_driveSubsystem = driveSubsystem;
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrupted) {
        m_driveSubsystem.tankDrive(0, 0);
    }
}