
package frc.robot.commands.drivecommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpiutil.math.MathUtil;
import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;

public class CurvatureDriveCommand extends CommandBase {

    private final DriveSubsystem m_driveSubsystem;
    private final Supplier<Double> m_speedStraight, m_speedLeft, m_speedRight;
    private double m_quickStopAccumulator = 0;

    public CurvatureDriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> speedStraight,
            Supplier<Double> speedLeft, Supplier<Double> speedRight) {
        m_driveSubsystem = driveSubsystem;
        m_speedStraight = speedStraight;
        m_speedLeft = speedLeft;
        m_speedRight = speedRight;
        addRequirements(m_driveSubsystem);
    }

    public void execute() {
        double speedStraight = Math.abs(m_speedStraight.get()) > ControllerConstants.kDeadzone ? m_speedStraight.get()
                : 0;
        double speedLeft = Math.abs(m_speedLeft.get()) > ControllerConstants.kTriggerDeadzone ? m_speedLeft.get() : 0;
        double speedRight = Math.abs(m_speedRight.get()) > ControllerConstants.kTriggerDeadzone ? m_speedRight.get()
                : 0;
        boolean isQuickTurn = Math.abs(speedStraight) == 0;
        curvatureDrive(speedStraight, speedRight - speedLeft, isQuickTurn);
    }

    public void curvatureDrive(double xSpeed, double zRotation, boolean isQuickTurn) {
        xSpeed = MathUtil.clamp(xSpeed, -1.0, 1.0);
        zRotation = MathUtil.clamp(zRotation, -1.0, 1.0);

        double angularPower;
        boolean overPower;

        if (isQuickTurn) {
            if (Math.abs(xSpeed) < DriveConstants.kQuickStopThreshold) {
                m_quickStopAccumulator = (1 - DriveConstants.kQuickStopAlpha) * m_quickStopAccumulator
                        + DriveConstants.kQuickStopAlpha * MathUtil.clamp(zRotation, -1.0, 1.0) * 2;
            }
            overPower = true;
            angularPower = zRotation;
        } else {
            overPower = false;
            angularPower = Math.abs(xSpeed) * zRotation - m_quickStopAccumulator;

            if (m_quickStopAccumulator > 1) {
                m_quickStopAccumulator -= 1;
            } else if (m_quickStopAccumulator < -1) {
                m_quickStopAccumulator += 1;
            } else {
                m_quickStopAccumulator = 0.0;
            }
        }

        double leftMotorOutput = xSpeed + angularPower;
        double rightMotorOutput = xSpeed - angularPower;

        // If rotation is overpowered, reduce both outputs to within acceptable range
        if (overPower) {
            if (leftMotorOutput > 1.0) {
                rightMotorOutput -= leftMotorOutput - 1.0;
                leftMotorOutput = 1.0;
            } else if (rightMotorOutput > 1.0) {
                leftMotorOutput -= rightMotorOutput - 1.0;
                rightMotorOutput = 1.0;
            } else if (leftMotorOutput < -1.0) {
                rightMotorOutput -= leftMotorOutput + 1.0;
                leftMotorOutput = -1.0;
            } else if (rightMotorOutput < -1.0) {
                leftMotorOutput -= rightMotorOutput + 1.0;
                rightMotorOutput = -1.0;
            }
        }
        double maxMagnitude = Math.max(Math.abs(leftMotorOutput), Math.abs(rightMotorOutput));
        if (maxMagnitude > 1.0) {
            leftMotorOutput /= maxMagnitude;
            rightMotorOutput /= maxMagnitude;
        }
        m_driveSubsystem.tankDrive(leftMotorOutput, rightMotorOutput);
    }

}