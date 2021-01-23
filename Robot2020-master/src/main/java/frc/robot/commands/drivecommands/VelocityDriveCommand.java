package frc.robot.commands.drivecommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class VelocityDriveCommand extends CommandBase {

        private final DriveSubsystem m_driveSubsystem;
        private final Supplier<Double> m_speedStraight, m_speedLeft, m_speedRight;

        /**
         * Drive using speed inputs as a velocity percentage out of a maximum velocity
         * 
         * @param driveSubsystem The subsystem to be used
         * @param speedStraight  Supplier of straight speed
         * @param speedLeft      Supplier of left speed
         * @param speedRight     Supplier of right speed
         */
        public VelocityDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedStraight,
                        Supplier<Double> speedLeft, Supplier<Double> speedRight) {
                m_driveSubsystem = driveSubsystem;
                m_speedStraight = speedStraight;
                m_speedLeft = speedLeft;
                m_speedRight = speedRight;
                addRequirements(m_driveSubsystem);
        }

        /**
         * Update the motor outputs
         */
        public void execute() {
                double speedStraight = Math.abs(m_speedStraight.get()) > ControllerConstants.kDeadzone
                                ? m_speedStraight.get()
                                : 0;
                double speedLeft = Math.abs(m_speedLeft.get()) > ControllerConstants.kTriggerDeadzone
                                ? m_speedLeft.get()
                                : 0;
                double speedRight = Math.abs(m_speedRight.get()) > ControllerConstants.kTriggerDeadzone
                                ? m_speedRight.get()
                                : 0;

                // Calculate robot velocity before calculating Drive wheel speeds
                DifferentialDriveWheelSpeeds wheelSpeeds = DriveConstants.kDriveKinematics.toWheelSpeeds(
                                new ChassisSpeeds(speedStraight * DriveConstants.kMaxSpeedMetersPerSecond, 0,
                                                (speedLeft - speedRight) * DriveConstants.kMaxRotSpeedMetersPerSecond));

                m_driveSubsystem.setWheelSpeeds(wheelSpeeds);
        }
}
