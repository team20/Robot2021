package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.CarouselSubsystem;

public class StopCarouselCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;

    /**
     * Stop the carousel
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public StopCarouselCommand(CarouselSubsystem carouselSubsystem) {
        m_carouselSubsystem = carouselSubsystem;
        addRequirements(m_carouselSubsystem);
    }

    /**
     * Stop the carousel
     */
    public void initialize() {
        m_carouselSubsystem.setVelocity(0);
    }
}