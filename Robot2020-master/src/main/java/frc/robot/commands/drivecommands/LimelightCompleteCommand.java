package frc.robot.commands.drivecommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldLocation;
import frc.robot.Constants.LimelightConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightCompleteCommand extends CommandBase {

    private final LimelightSubsystem m_limelightSubsystem;
    private final DriveSubsystem m_driveSubsystem;
    private final double m_turnGoal, m_distanceGoal;
    private final ProfiledPIDController m_turnController = new ProfiledPIDController(LimelightConstants.kTurnP,
            LimelightConstants.kTurnI, LimelightConstants.kTurnD, new Constraints(
                    DriveConstants.kMaxRotSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared));
    private final ProfiledPIDController m_distanceController = new ProfiledPIDController(LimelightConstants.kDisP,
            LimelightConstants.kDisI, LimelightConstants.kDisD, new Constraints(DriveConstants.kMaxSpeedMetersPerSecond,
                    DriveConstants.kMaxAccelerationMetersPerSecondSquared));

    /**
     * Use the limelight to both reach a desired distance and angle to the powerport
     * 
     * @param limelightSubsystem The limelight subsystem to gather data from
     * @param driveSubsystem     The train subsystem to be used
     * @param turnGoal           Angle setpoint towards the target
     * @param distanceGoal       Distance goal away from the target
     */
    public LimelightCompleteCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem driveSubsystem,
            double turnGoal, double distanceGoal) {
        m_limelightSubsystem = limelightSubsystem;
        m_driveSubsystem = driveSubsystem;
        m_turnGoal = turnGoal;
        m_distanceGoal = distanceGoal;
        addRequirements(m_driveSubsystem);
    }

    /**
     * Use the limelight to both reach a desired distance and angle to the powerport
     * 
     * @param limelightSubsystem  The limelight subsystem to gather data from
     * @param drivetrainSubsystem The drivetrain subsystem to be used
     * @param fieldLocation       A supplier for a location on the field
     */
    public LimelightCompleteCommand(LimelightSubsystem limelightSubsystem, DriveSubsystem drivetrainSubsystem,
            Supplier<FieldLocation> fieldLocation) {
        m_limelightSubsystem = limelightSubsystem;
        m_driveSubsystem = drivetrainSubsystem;
        m_turnGoal = fieldLocation.get().turnGoal;
        m_distanceGoal = fieldLocation.get().distanceGoal;
        addRequirements(m_driveSubsystem);
    }

    /**
     * Set the tolerance and goal of each PID
     */
    public void initialize() {
        m_turnController.setTolerance(LimelightConstants.kTurnTolerance);
        m_turnController.setGoal(m_turnGoal);

        m_distanceController.setTolerance(LimelightConstants.kDistanceTolerance);
        m_distanceController.setGoal(m_distanceGoal);
    }

    /**
     * Update the motor outputs
     */
    public void execute() {
        double robotTurnSpeed = m_turnController.calculate(m_limelightSubsystem.getXAngle());
        double robotTranslationSpeed = m_distanceController.calculate(m_limelightSubsystem.getDistance());
        DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics
                .toWheelSpeeds(new ChassisSpeeds(robotTranslationSpeed, 0, robotTurnSpeed));
        m_driveSubsystem.setWheelSpeeds(wheelSpeeds);
    }

    /**
     * Stop the drivetrain at the end of the command
     */
    public void end(boolean interrputed) {
        m_driveSubsystem.tankDrive(0, 0);
    }

    /**
     * End the command when both PIDs are at their setpoints
     */
    public boolean isFinished() {
        return m_turnController.atSetpoint() && m_distanceController.atSetpoint();
    }
}