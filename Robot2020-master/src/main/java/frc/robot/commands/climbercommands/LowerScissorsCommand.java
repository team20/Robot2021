package frc.robot.commands.climbercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberSubsystem;

public class LowerScissorsCommand extends CommandBase {

    private final ClimberSubsystem m_climberSubsystem;

    /**
     * Lower the climber back to the robot
     * 
     * @param climberSubsystem The climber subsystem to be used
     */
    public LowerScissorsCommand(ClimberSubsystem climberSubsystem) {
        m_climberSubsystem = climberSubsystem;
        addRequirements(m_climberSubsystem);
    }

    /**
     * Update the climber setpoint
     */
    public void initialize() {
        m_climberSubsystem.setPosition(0);
    }
}