package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class ExtendArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    /**
     * Extend the arm
     * 
     * @param armSubsystem {@link ArmSubsystem} to be used.
     */
    public ExtendArmCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    /**
     * Update arm setpoint
     */
    public void initialize() {
        m_armSubsystem.setPosition(ArmConstants.kOutPosition);
    }
}