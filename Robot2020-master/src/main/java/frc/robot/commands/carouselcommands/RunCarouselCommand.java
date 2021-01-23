package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarouselSubsystem;

public class RunCarouselCommand extends CommandBase {

	private final CarouselSubsystem m_carouselSubsystem;
	private final double m_speed;

	/**
	 * Run the carousel at normal speed
	 * 
	 * @param carouselSubsystem {@link CarouselSubsystem} to be used.
	 */
	public RunCarouselCommand(CarouselSubsystem carouselSubsystem, double speed) {
		m_carouselSubsystem = carouselSubsystem;
		m_speed = speed;
		addRequirements(m_carouselSubsystem);
	}

	/**
	 * Run the carousel
	 */
	public void initialize() {
		m_carouselSubsystem.setVelocity(m_speed);
	}

	/**
	 * Stop the carousel at the end of the command
	 */
	public void end(boolean interrupted) {
		m_carouselSubsystem.setVelocity(0.0);
	}
}