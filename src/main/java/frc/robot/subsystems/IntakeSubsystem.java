package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.ShuffleboardLogging;

public class IntakeSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final VictorSPX m_motor = new VictorSPX(IntakeConstants.kMotorPort);

	/**
	 * Initializes a new instance of the {@link IntakeSubsystem} class.
	 */
	public IntakeSubsystem() {
		m_motor.setNeutralMode(NeutralMode.Coast);
		m_motor.enableVoltageCompensation(true);
		m_motor.setInverted(IntakeConstants.kInvert);
	}

	/**
	 * Sets new speed for the intake wheel to spin at.
	 * 
	 * @param speed Percent output.
	 */
	public void setSpeed(double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
	}

	public void configureShuffleboard() {
		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Intake");
		shuffleboardTab.addNumber("Motor output", () -> m_motor.getMotorOutputPercent()).withSize(4, 2)
				.withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
	}
}