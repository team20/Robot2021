package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.AccelStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.ShuffleboardLogging;

public class ClimberSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(ClimberConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_pidController = m_motor.getPIDController();
    private double m_setPosition = 0;

    /**
     * Initializes a new instance of the {@link ClimberSubsystem} class.
     */
    public ClimberSubsystem() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(ClimberConstants.kInvert);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(ClimberConstants.kSmartCurrentLimit);

        m_pidController.setP(ClimberConstants.kP);
        m_pidController.setI(ClimberConstants.kI);
        m_pidController.setIZone(ClimberConstants.kIz);
        m_pidController.setD(ClimberConstants.kD);
        m_pidController.setFF(ClimberConstants.kFF);
        m_pidController.setOutputRange(ClimberConstants.kMinOutput, ClimberConstants.kMaxOutput);

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ClimberConstants.kSlotID);
        m_pidController.setSmartMotionMaxAccel(ClimberConstants.kMaxAcel, ClimberConstants.kSlotID);
        m_pidController.setSmartMotionMaxVelocity(ClimberConstants.kMaxVelocity, ClimberConstants.kSlotID);
        m_pidController.setSmartMotionAllowedClosedLoopError(ClimberConstants.kAllowedError, ClimberConstants.kSlotID);
        m_pidController.setSmartMotionMinOutputVelocity(ClimberConstants.kMinVelocity, ClimberConstants.kSlotID);
    }

    /**
     * @return Current climber position (motor rotations)
     */
    public double getPosition() {
        return m_encoder.getPosition();
    }

    /**
     * @return Current velocity (motor rotations/s)
     */
    public double getVelocity() {
        return m_encoder.getVelocity();
    }

    /**
     * @return Whether the climber is at the setpoint
     */
    public boolean atSetpoint() {
        return (Math.abs(-m_setPosition - getPosition()) <= ClimberConstants.kAllowedError);
    }

    /**
     * @param speed Percent output of the arm
     */
    public void setPercentOutput(double speed) {
        m_motor.set(speed);
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setPosition(double position) {
        m_setPosition = position;
        m_pidController.setReference(position, ControlType.kSmartMotion, ClimberConstants.kSlotID);
    }

    /**
     * Zero the encoder position
     */
    public void resetEncoder() {
        m_encoder.setPosition(0);
    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Climber");
        shuffleboardTab.addNumber("Encoder Position", () -> getPosition()).withSize(4, 2).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Encoder Velocity", () -> getVelocity()).withSize(4, 2).withPosition(4, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Output", () -> m_motor.getAppliedOutput()).withSize(1, 1).withPosition(1, 2)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addBoolean("At setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}