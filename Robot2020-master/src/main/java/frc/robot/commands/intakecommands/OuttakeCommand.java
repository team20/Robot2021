package frc.robot.commands.intakecommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class OuttakeCommand extends CommandBase {

    private final IntakeSubsystem m_intakeSubsystem;

    /**
     * Run the intake motor backwards
     * 
     * @param intakeSubsystem {@link IntakeSubsystem} to be used.
     */
    public OuttakeCommand(IntakeSubsystem intakeSubsystem) {
        m_intakeSubsystem = intakeSubsystem;
        addRequirements(m_intakeSubsystem);
    }

    /**
     * Run the motor backwards
     */
    public void initialize() {
        m_intakeSubsystem.setSpeed(-IntakeConstants.kPercentOutput);
    }

    /**
     * Stop the intake at the end of the command
     */
    public void end(boolean interrupted) {
        m_intakeSubsystem.setSpeed(0);
    }
}