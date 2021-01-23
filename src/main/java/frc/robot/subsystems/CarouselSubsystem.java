package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CarouselConstants;
import frc.robot.ShuffleboardLogging;

public class CarouselSubsystem extends SubsystemBase implements ShuffleboardLogging {

	private final CANSparkMax m_motor = new CANSparkMax(CarouselConstants.kMotorPort, MotorType.kBrushless);
	private final CANEncoder m_encoder = m_motor.getEncoder();
	private final CANPIDController m_pidController = m_motor.getPIDController();
	private final DigitalInput m_magSensor = new DigitalInput(CarouselConstants.kMagSensorPort);
	private double m_lastSensorPosition;

	/**
	 * Initializes a new instance of the {@link CarouselSubsystem} class.
	 */
	public CarouselSubsystem() {
		m_motor.restoreFactoryDefaults();
		m_motor.setInverted(CarouselConstants.kInvert);
		m_motor.setIdleMode(IdleMode.kCoast);
		m_motor.enableVoltageCompensation(12);
		m_motor.setSmartCurrentLimit(CarouselConstants.kSmartCurrentLimit);

		m_encoder.setPositionConversionFactor(1 / CarouselConstants.kGearRatio);
		m_encoder.setVelocityConversionFactor(1 / CarouselConstants.kGearRatio);

		m_pidController.setP(CarouselConstants.kP);
		m_pidController.setI(CarouselConstants.kI);
		m_pidController.setIZone(CarouselConstants.kIz);
		m_pidController.setD(CarouselConstants.kD);
		m_pidController.setFF(CarouselConstants.kFF);
		m_pidController.setOutputRange(CarouselConstants.kMinOutput, CarouselConstants.kMaxOutput);

		resetEncoder();
	}

	public void periodic() {
		System.out.println("Sensor: " + m_magSensor.get());
		if (!m_magSensor.get()) {
			m_lastSensorPosition = getPosition();
		}
		SmartDashboard.putNumber("Carousel Velocity", m_encoder.getVelocity());
	}

	/**
	 * @return Position of encoder (carousel rotations).
	 */
	public double getPosition() {
		return m_encoder.getPosition();
	}

	/**
	 * @return Measured velocity of the carousel (rpm).
	 */
	public double getVelocity() {
		return m_encoder.getVelocity();
	}

	public double getLastSensorPosition() {
		return m_lastSensorPosition;
	}

	public boolean atOpenSpace() {
		return getPosition() - m_lastSensorPosition < CarouselConstants.kStartPositionTolerance;
	}

	public void setPosition(double position) {
		m_pidController.setP(CarouselConstants.kPositionP);
		m_pidController.setI(CarouselConstants.kPositionI);
		m_pidController.setIZone(CarouselConstants.kPositionIz);
		m_pidController.setD(CarouselConstants.kPositionD);
		m_pidController.setFF(CarouselConstants.kPositionFF);

		m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, CarouselConstants.kSlotID);
		m_pidController.setSmartMotionMaxAccel(CarouselConstants.kMaxAcel, CarouselConstants.kSlotID);
		m_pidController.setSmartMotionMaxVelocity(CarouselConstants.kMaxVelocity, CarouselConstants.kSlotID);
		m_pidController.setSmartMotionAllowedClosedLoopError(CarouselConstants.kAllowedError,
				CarouselConstants.kSlotID);
		m_pidController.setSmartMotionMinOutputVelocity(CarouselConstants.kMinVelocity, CarouselConstants.kSlotID);

		m_pidController.setReference(position, ControlType.kSmartMotion);
	}

	/**
	 * Set new velocity for the carousel to spin at.
	 * 
	 * @param velocity Motor rpm.
	 */
	public void setVelocity(double velocity) {
		m_pidController.setP(CarouselConstants.kP);
		m_pidController.setI(CarouselConstants.kI);
		m_pidController.setIZone(CarouselConstants.kIz);
		m_pidController.setD(CarouselConstants.kD);
		m_pidController.setFF(CarouselConstants.kFF);
		
		if (velocity == 0.0) {
			m_motor.set(0);
		} else {
			m_pidController.setReference(velocity, ControlType.kVelocity);
		}
	}

	/**
	 * Zero the encoder position
	 */
	public void resetEncoder() {
		m_encoder.setPosition(0);
	}

	public void configureShuffleboard() {
		ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Carousel");
		shuffleboardTab.addNumber("Encoder Velocity", () -> getVelocity()).withSize(4, 2).withPosition(0, 0)
				.withWidget(BuiltInWidgets.kGraph);
		shuffleboardTab.addNumber("Output", () -> m_motor.getAppliedOutput()).withSize(1, 1).withPosition(0, 2)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addBoolean("At Open Space", () -> atOpenSpace()).withSize(1, 1).withPosition(1, 2)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addNumber("Current", () -> m_motor.getOutputCurrent()).withSize(1, 1).withPosition(2, 2)
				.withWidget(BuiltInWidgets.kTextView);
		shuffleboardTab.addNumber("Position", () -> m_encoder.getPosition()).withSize(1, 1).withPosition(3, 2)
				.withWidget(BuiltInWidgets.kTextView);
	}
}