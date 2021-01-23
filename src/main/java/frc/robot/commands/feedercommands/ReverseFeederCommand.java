package frc.robot.commands.feedercommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class ReverseFeederCommand extends CommandBase {

	private final FeederSubsystem m_feederSubsystem;

	/**
	 * Begin the feeder immedietly
	 * 
	 * @param feederSubsystem {@link FeederSubsystem} to be used.
	 */
	public ReverseFeederCommand(FeederSubsystem feederSubsystem) {
		m_feederSubsystem = feederSubsystem;
		addRequirements(m_feederSubsystem);
	}

	/**
	 * Update the motor output
	 */
	public void initialize() {
		m_feederSubsystem.setPercentOutput(-FeederConstants.kSpeed);
	}

	/**
	 * Stop the feeder at the end of the command
	 */
	public void end(boolean interrupted) {
		m_feederSubsystem.setPercentOutput(0.0);
	}
}