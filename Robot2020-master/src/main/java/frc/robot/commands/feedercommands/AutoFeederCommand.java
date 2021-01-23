package frc.robot.commands.feedercommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.subsystems.FeederSubsystem;

public class AutoFeederCommand extends CommandBase {

	private final FeederSubsystem m_feederSubsystem;
	private final Supplier<Boolean> m_carouselReady;
	private final Supplier<Boolean> m_flywheelReady;

	/**
	 * Begin the feeder when the carousel is at the feeder opening and the flywheel
	 * is at speed
	 * 
	 * @param feederSubsystem  {@link FeederSubsystem} to be used.
	 * @param carouselPosition The current position of the carousel
	 * @param flywheelReady    Whether the flywheel is at speed
	 */
	public AutoFeederCommand(FeederSubsystem feederSubsystem, Supplier<Boolean> carouselReady,
			Supplier<Boolean> flywheelReady) {
		m_feederSubsystem = feederSubsystem;
		m_carouselReady = carouselReady;
		m_flywheelReady = flywheelReady;
		addRequirements(m_feederSubsystem);
	}

	/**
	 * Run feeder motor at correct carousel position
	 */
	public void execute() {
		if (m_carouselReady.get() && m_flywheelReady.get()) {
			m_feederSubsystem.setPercentOutput(FeederConstants.kSpeed);
		}
	}

	/**
	 * Stop the feeder at the end of the command
	 */
	public void end(boolean interrupted) {
		m_feederSubsystem.setPercentOutput(0.0);
	}
}