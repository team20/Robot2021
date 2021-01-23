package frc.robot.commands.shootcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FieldLocation;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ShootSetupCommand extends CommandBase {

    private final FlywheelSubsystem m_flywheelSubsystem;
    private final HoodSubsystem m_hoodSubsystem;
    private final double m_flywheelSetpoint, m_hoodSetpoint;

    /**
     * Setup the flywheel and hood
     * 
     * @param flywheelSubsystem The flywheel subsystem to be used
     * @param hoodSubsystem     The hood subsystem to be used
     * @param flywheelSetpoint  The flywheel setpoint
     * @param hoodSetpoint      The hood setpoint
     */
    public ShootSetupCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, double flywheelSetpoint,
            double hoodSetpoint) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_flywheelSetpoint = flywheelSetpoint;
        m_hoodSetpoint = hoodSetpoint;
        addRequirements(m_flywheelSubsystem, m_hoodSubsystem);
    }

    /**
     * Setup the flywheel and hood
     * 
     * @param flywheelSubsystem The flywheel subsystem to be used
     * @param hoodSubsystem     The hood subsystem to be used
     * @param fieldLocation     The field location to get other setpoints from
     */
    public ShootSetupCommand(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem,
            Supplier<FieldLocation> fieldLocation) {
        m_flywheelSubsystem = flywheelSubsystem;
        m_hoodSubsystem = hoodSubsystem;
        m_flywheelSetpoint = fieldLocation.get().flywheelSetpoint;
        m_hoodSetpoint = fieldLocation.get().hoodSetpoint;
        addRequirements(m_flywheelSubsystem, m_hoodSubsystem);
    }

    /**
     * Set the setpoints of the flywheel and hood
     */
    public void initialize() {
        m_flywheelSubsystem.setVelocity(m_flywheelSetpoint);
        m_hoodSubsystem.setPosition(m_hoodSetpoint);
    }

    /**
     * Stop the flywheel and reset the hood at the end of the command
     */
    public void end(boolean interrupted) {
        m_flywheelSubsystem.setVelocity(0);
        m_hoodSubsystem.setPosition(0);
    }
}
