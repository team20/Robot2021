package frc.robot.commands.shootcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightShootSetupCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final LimelightSubsystem m_limelightSubsystem;

    /**
     * Set flywheel and hood setpoints from distance calculated from the limelight
     * 
     * @param flywheelSubsystem  The flywheel subsystem to be used
     * @param hoodSubsystem      The hood subsystem to be used
     * @param limelightSubsystem The limelight subsystem to be used
     */
    public LimelightShootSetupCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem,
            LimelightSubsystem limelightSubsystem) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_limelightSubsystem = limelightSubsystem;
        addRequirements(m_flywheelSubsystem, m_hoodSubsystem);
    }

    /**
     * Update the setpoints of the hood and flywheel using limelight data
     */
    public void initialize() {
        // TODO - Kevin do math here - Calculate optimal hood angle and flywheel speed
        // given a distance from the limelight
    }

    /**
     * Stop the flywheel after the balls are shot
     */
    public void end(boolean interrupted) {
        m_flywheelSubsystem.setVelocity(0);
    }
}
