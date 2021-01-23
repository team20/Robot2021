package frc.robot.commands.armcommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class BounceArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private double lastBounceTime;
    private boolean armDown;

    /**
     * Bounce the arm to improve intaking
     * 
     * @param armSubsystem {@link ArmSubsystem} to be used.
     */
    public BounceArmCommand(ArmSubsystem armSubsystem) {
        m_armSubsystem = armSubsystem;
        addRequirements(m_armSubsystem);
    }

    public void initialize() {
        lastBounceTime = Timer.getFPGATimestamp();
        m_armSubsystem.setPosition(ArmConstants.kBounceDownPosition);
        armDown = true;
    }

    public void execute() {
        if (Timer.getFPGATimestamp() - lastBounceTime >= ArmConstants.kBounceTime) {
            if (armDown) {
                m_armSubsystem.setPosition(ArmConstants.kBounceUpPosition);
            } else {
                m_armSubsystem.setPosition(ArmConstants.kBounceDownPosition);
            }
            armDown = !armDown;
            lastBounceTime = Timer.getFPGATimestamp();
        }
    }

    public void end(boolean interrupted) {
        m_armSubsystem.setPosition(ArmConstants.kOutPosition);
    }
}