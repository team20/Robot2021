package frc.robot.commands.autocommands;

import java.nio.file.Path;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.CarouselConstants;
import frc.robot.commands.armcommands.BounceArmCommand;
import frc.robot.commands.carouselcommands.RunCarouselCommand;
import frc.robot.commands.drivecommands.TrajectoryFollowCommand;
import frc.robot.commands.intakecommands.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.CarouselSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShootTrenchCG extends SequentialCommandGroup {

    /**
     * Shoot balls move forward off init
     */
    public ShootTrenchCG(DriveSubsystem driveSubsystem, FlywheelSubsystem flywheelSubsystem,
            HoodSubsystem hoodSubsystem, FeederSubsystem feederSubsystem, CarouselSubsystem carouselSubsystem,
            IntakeSubsystem intakeSubsystem, ArmSubsystem armSubsystem) {

        try {
            // Move to trench
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(AutoConstants.kInitTrench);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            Command moveAndIntake = deadline(new TrajectoryFollowCommand(driveSubsystem, trajectory),
                    new IntakeCommand(intakeSubsystem), new BounceArmCommand(armSubsystem),
                    new RunCarouselCommand(carouselSubsystem, CarouselConstants.kIntakeVelocity));
            addCommands(new ShootCG(flywheelSubsystem, hoodSubsystem, feederSubsystem, carouselSubsystem),
                    moveAndIntake);
        } catch (Exception e) {

        }
    }
}