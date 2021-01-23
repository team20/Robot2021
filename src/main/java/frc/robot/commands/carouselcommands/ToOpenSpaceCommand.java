package frc.robot.commands.carouselcommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.subsystems.CarouselSubsystem;

public class ToOpenSpaceCommand extends CommandBase {

    private final CarouselSubsystem m_carouselSubsystem;

    /**
     * Set the carousel to be ready for shooting whenever it is not running
     * 
     * @param carouselSubsystem {@link CarouselSubsystem} to be used.
     */
    public ToOpenSpaceCommand(CarouselSubsystem carouselSubsystem) {
        m_carouselSubsystem = carouselSubsystem;
        addRequirements(m_carouselSubsystem);
    }

    public void execute() {
        if (m_carouselSubsystem.atOpenSpace()) {
            m_carouselSubsystem.setVelocity(0);
        } else {
            m_carouselSubsystem.setVelocity(CarouselConstants.kVelocity);
        }
    }

    /**
     * Stop the carousel at the end of the command
     */
    public void end(boolean interrupted) {
        m_carouselSubsystem.setVelocity(0.0);
    }
}