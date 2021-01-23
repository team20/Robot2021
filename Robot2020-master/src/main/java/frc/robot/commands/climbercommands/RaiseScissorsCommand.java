package frc.robot.commands.climbercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class RaiseScissorsCommand extends CommandBase {

    private final ClimberSubsystem m_climberSubsystem;

    /**
     * Raise the climber to the correct level
     * 
     * @param climberSubsystem The climber subsystem to be used
     */
    public RaiseScissorsCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
        addRequirements(m_climberSubsystem);
    }

    /**
     * Update the climber setpoint
     */
    public void initialize() {
        m_climberSubsystem.setPosition(ClimberConstants.kTopSetpoint);
    }
}