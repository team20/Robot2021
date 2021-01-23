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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.HoodConstants;
import frc.robot.ShuffleboardLogging;

public class HoodSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(HoodConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_pidController = m_motor.getPIDController();
    private double m_setPosition = 0;

    /**
     * Initializes a new instance of the {@link HoodSubsystem} class.
     */
    public HoodSubsystem() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(HoodConstants.kInvert);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(HoodConstants.kSmartCurrentLimit);

        m_pidController.setP(HoodConstants.kP);
        m_pidController.setI(HoodConstants.kI);
        m_pidController.setIZone(HoodConstants.kIz);
        m_pidController.setD(HoodConstants.kD);
        m_pidController.setFF(HoodConstants.kFF);
        m_pidController.setOutputRange(HoodConstants.kMinOutput, HoodConstants.kMaxOutput);

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, HoodConstants.kSlotID);
        m_pidController.setSmartMotionMaxAccel(HoodConstants.kMaxAcel, HoodConstants.kSlotID);
        m_pidController.setSmartMotionMaxVelocity(HoodConstants.kMaxVelocity, HoodConstants.kSlotID);
        m_pidController.setSmartMotionAllowedClosedLoopError(HoodConstants.kAllowedError, HoodConstants.kSlotID);
        m_pidController.setSmartMotionMinOutputVelocity(HoodConstants.kMinVelocity, HoodConstants.kSlotID);

        resetEncoder();
        setPosition(0);
    }

    public void periodic() {
        SmartDashboard.putNumber("Hood position", getPosition());
    }

    /**
     * @return Current position (motor rotations)
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
     * @return The angle of the hood from horizontal
     */
    public double getAngle() {
        return getPosition() * ((HoodConstants.kMaxAngle - HoodConstants.kMinAngle)
                / (HoodConstants.kMaxEncoderValue - HoodConstants.kMinEncoderValue)) + HoodConstants.kMinAngle;
    }

    /**
     * @return Whether the hood is at the setpoint
     */
    public boolean atSetpoint() {
        return (Math.abs(m_setPosition - getPosition()) <= HoodConstants.kAllowedError);
    }

    /**
     * @param speed Percent output of the hood
     */
    public void setPercentOutput(Double speed) {
        m_motor.set(speed);
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setPosition(double position) {
        m_setPosition = position;
        m_pidController.setReference(position, ControlType.kSmartMotion, HoodConstants.kSlotID, 0);
    }

    public void setSetpointDegrees(double degrees) {
        setPosition(
                (degrees - HoodConstants.kMinAngle) * ((HoodConstants.kMaxEncoderValue - HoodConstants.kMinEncoderValue)
                        / (HoodConstants.kMaxAngle - HoodConstants.kMinAngle)));
    }

    /**
     * Zero the encoder position
     */
    public void resetEncoder() {
        m_encoder.setPosition(0);
        setPosition(0);
    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Hood");
        shuffleboardTab.addNumber("Encoder Position", () -> getPosition()).withSize(4, 2).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Encoder Velocity", () -> getVelocity()).withSize(4, 2).withPosition(4, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addBoolean("At setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}