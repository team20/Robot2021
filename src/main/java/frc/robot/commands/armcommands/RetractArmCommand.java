package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class RetractArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;

    /**
     * Retract the arm
     * 
     * @param armSubsystem {@link ArmSubsystem} to be used.
     */
    public RetractArmCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    /**
     * Update arm setpoint
     */
    public void initialize() {
        m_armSubsystem.setPosition(ArmConstants.kInPosition);
    }
}