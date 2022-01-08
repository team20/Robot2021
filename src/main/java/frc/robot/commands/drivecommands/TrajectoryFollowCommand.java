package frc.robot.commands.drivecommands;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
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
        // super(trajectory, driveSubsystem::getPose, new RamseteController(), DriveConstants.kDriveKinematics,
        //         (left, right) -> driveSubsystem.tankDriveVolts(left, right),
        //         driveSubsystem);
        // m_driveSubsystem = driveSubsystem;
        //System.out.println("trajectory follow command is running.");

        super(trajectory, driveSubsystem::getPose, new RamseteController(), DriveConstants.kDriveKinematics,
                (left, right) -> driveSubsystem.setWheelSpeeds(new DifferentialDriveWheelSpeeds(left, right)),
                driveSubsystem);
        m_driveSubsystem = driveSubsystem;

        // Create config for trajectory
        // RamseteCommand ramseteCommand = new RamseteCommand(trajectory, m_driveSubsystem::getPose,
        //         new RamseteController(),
        //         new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter,
        //                 DriveConstants.kaVoltSecondsSquaredPerMeter),
        //         DriveConstants.kDriveKinematics, driveSubsystem::getWheelSpeeds,
        //         new PIDController(DriveConstants.kP, 0, 0), new PIDController(DriveConstants.kP, 0, 0),
        //         // RamseteCommand passes volts to the callback
        //         (left, right) -> driveSubsystem.tankDriveVolts(left, right), m_driveSubsystem);

        // Reset odometry to the starting pose of the trajectory.
        m_driveSubsystem.resetOdometry(trajectory.getInitialPose());

        // Run path following command, then stop at the end.
        //ramseteCommand.andThen(() -> m_driveSubsystem.tankDriveVolts(0, 0));
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrupted) {
        m_driveSubsystem.tankDrive(0, 0);
    }
}