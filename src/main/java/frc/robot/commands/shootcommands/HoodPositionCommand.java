package frc.robot.commands.shootcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.HoodSubsystem;

public class HoodPositionCommand extends CommandBase {

    private final HoodSubsystem m_hoodSubsystem;
    private final double m_setpoint;

    /**
     * Drive the hood using setpoints
     * 
     * @param hoodSubsystem The hood subsystem to be used
     * @param setpoint      The desired encoder position
     */
    public HoodPositionCommand(HoodSubsystem hoodSubsystem, double setpoint) {
        m_hoodSubsystem = hoodSubsystem;
        m_setpoint = setpoint;
        addRequirements(m_hoodSubsystem);
    }

    /**
     * Update the setpoint
     */
    public void execute() {
        m_hoodSubsystem.setPosition(m_setpoint);
    }
}