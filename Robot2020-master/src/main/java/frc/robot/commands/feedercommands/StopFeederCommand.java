package frc.robot.commands.feedercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FeederSubsystem;

public class StopFeederCommand extends CommandBase {

    private final FeederSubsystem m_feederSubsystem;

    /**
     * Stop the feeder
     * 
     * @param feederSubsystem {@link FeederSubsystem} to be used.
     */
    public StopFeederCommand(FeederSubsystem feederSubsystem) {
        m_feederSubsystem = feederSubsystem;
        addRequirements(m_feederSubsystem);
    }

    /**
     * Stop the motor
     */
    public void initialize() {
        m_feederSubsystem.setPercentOutput(0);
    }
}