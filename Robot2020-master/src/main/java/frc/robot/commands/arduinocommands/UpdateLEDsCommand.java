package frc.robot.commands.arduinocommands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArduinoConstants;
import frc.robot.subsystems.ArduinoSubsystem;

public class UpdateLEDsCommand extends CommandBase {
    private ArduinoSubsystem m_arduinoSubsystem;
    private Supplier<Byte> m_mainLEDMode;
    private Supplier<Double> m_mainLEDValue;
    private Supplier<Byte> m_shooterLEDMode;
    private Supplier<Double> m_shooterLEDValue;

    public UpdateLEDsCommand(ArduinoSubsystem arduinoSubsystem, Supplier<Byte> mainLEDMode, Supplier<Double> mainLEDValue, Supplier<Byte> shooterLEDMode, Supplier<Double> shooterLEDValue) {
        m_arduinoSubsystem = arduinoSubsystem;
        m_mainLEDMode = mainLEDMode;
        m_mainLEDValue = mainLEDValue;
        m_shooterLEDMode = shooterLEDMode;
        m_shooterLEDValue = shooterLEDValue;
        addRequirements(m_arduinoSubsystem);
    }

	@Override
    public void execute() {
        m_arduinoSubsystem.setMainLEDMode(m_mainLEDMode.get());
        m_arduinoSubsystem.setMainLEDValue(m_mainLEDValue.get());
        m_arduinoSubsystem.setShooterLEDMode(m_shooterLEDMode.get());
        m_arduinoSubsystem.setShooterLEDValue(m_shooterLEDValue.get());
    }

    @Override
    public void end(boolean interrupted) {
        m_arduinoSubsystem.setMainLEDMode(ArduinoConstants.MainLEDModes.kOff);
        m_arduinoSubsystem.setShooterLEDMode(ArduinoConstants.ShooterLEDModes.kOff);
    }
}