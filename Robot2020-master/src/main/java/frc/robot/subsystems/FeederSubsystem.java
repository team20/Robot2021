package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FeederConstants;
import frc.robot.ShuffleboardLogging;

public class FeederSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final VictorSPX m_motor = new VictorSPX(FeederConstants.kMotorPort);

	/**
	 * Initializes a new instance of the {@link FeederSubsystem} class.
	 */
	public FeederSubsystem() {
		m_motor.setNeutralMode(NeutralMode.Coast);
		m_motor.enableVoltageCompensation(true);
		m_motor.setInverted(FeederConstants.kInvert);
	}

	public double getPercentOutput() {
		return m_motor.getMotorOutputPercent();
	}

	/**
	 * Sets new speed for the feeder wheel to spin at.
	 * 
	 * @param speed Percent output.
	 */
	public void setPercentOutput(double speed) {
		m_motor.set(ControlMode.PercentOutput, speed);
	}

	public void configureShuffleboard() {
		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Feeder");
		shuffleboardTab.addNumber("Motor output", () -> m_motor.getMotorOutputPercent()).withSize(4, 2)
				.withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
	}
}