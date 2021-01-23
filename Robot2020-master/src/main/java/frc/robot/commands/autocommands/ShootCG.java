package frc.robot.commands.autocommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FieldLocation;
import frc.robot.commands.carouselcommands.AutoSpeedCarouselCommand;
import frc.robot.commands.feedercommands.AutoFeederCommand;
import frc.robot.commands.shootcommands.ShootSetupCommand;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class ShootCG extends ParallelRaceGroup {

    /**
     * Shoot balls
     */
    public ShootCG(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem,
            CarouselSubsystem carouselSubsystem) {
        // Sping flywheel, set hood, auto feed, and shoot
        Command shootSetup = parallel(
                new ShootSetupCommand(flywheelSubsystem, hoodSubsystem, () -> FieldLocation.INITLINE),
                new AutoFeederCommand(feederSubsystem, carouselSubsystem::atOpenSpace, flywheelSubsystem::atSetpoint));
        Command shootCommand = sequence(new WaitUntilCommand(() -> feederSubsystem.getPercentOutput() > 0),
                new AutoSpeedCarouselCommand(carouselSubsystem, flywheelSubsystem::getSetpoint)).withTimeout(6);
        addCommands(shootSetup, shootCommand);

    }
}