package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Run the intake motor forwards
     * 
     * @param intakeSubsystem {@link IntakeSubsystem} to be used.
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    /**
     * Run the motor forwards
     */
    public void initialize() {
        m_intakeSubsystem.setSpeed(IntakeConstants.kPercentOutput);
    }

    /**
     * Stop the intake at the end of the command
     */
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}