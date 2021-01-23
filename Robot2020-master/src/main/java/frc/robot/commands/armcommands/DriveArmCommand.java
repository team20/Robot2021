package frc.robot.commands.armcommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.ArmSubsystem;

public class DriveArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final Supplier<Double> m_reverseSpeed, m_forwardSpeed;

    /**
     * Drive the arm using percent output
     * 
     * @param armSubsystem The hood subsystem to be used
     * @param reverseSpeed Supplier of reverse speed (retract)
     * @param forwardSpeed Supplier of forward speed (extend)
     */
    public DriveArmCommand(ArmSubsystem armSubsystem, Supplier<Double> reverseSpeed, Supplier<Double> forwardSpeed) {
        m_armSubsystem = armSubsystem;
        m_reverseSpeed = reverseSpeed;
        m_forwardSpeed = forwardSpeed;
        addRequirements(m_armSubsystem);
    }

    /**
     * Update the motor output
     */
    public void execute() {
        double reverseSpeed = Math.abs(m_reverseSpeed.get()) > ControllerConstants.kTriggerDeadzone
                ? m_reverseSpeed.get()
                : 0;
        double forwardSpeed = Math.abs(m_forwardSpeed.get()) > ControllerConstants.kTriggerDeadzone
                ? m_forwardSpeed.get()
                : 0;
        m_armSubsystem.setPercentOutput(reverseSpeed - forwardSpeed);
    }

    /**
     * Stop the arm when the command ends
     */
    public void end(boolean interrupted) {
        m_armSubsystem.setPercentOutput(0);
    }
}