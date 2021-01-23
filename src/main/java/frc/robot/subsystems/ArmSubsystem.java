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
import frc.robot.Constants.ArmConstants;
import frc.robot.ShuffleboardLogging;

public class ArmSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_motor = new CANSparkMax(ArmConstants.kMotorPort, MotorType.kBrushless);
    private final CANEncoder m_encoder = m_motor.getEncoder();
    private final CANPIDController m_pidController = m_motor.getPIDController();
    private double m_setPosition = 0;

    /**
     * Initializes a new instance of the {@link ArmSubsystem} class.
     */
    public ArmSubsystem() {
        m_motor.restoreFactoryDefaults();
        m_motor.setInverted(ArmConstants.kInvert);
        m_motor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_motor.enableVoltageCompensation(12);
        m_motor.setSmartCurrentLimit(ArmConstants.kSmartCurrentLimit);

        m_pidController.setP(ArmConstants.kP);
        m_pidController.setI(ArmConstants.kI);
        m_pidController.setIZone(ArmConstants.kIz);
        m_pidController.setD(ArmConstants.kD);
        m_pidController.setFF(ArmConstants.kFF);
        m_pidController.setOutputRange(ArmConstants.kMinOutput, ArmConstants.kMaxOutput);

        m_pidController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMaxAccel(ArmConstants.kMaxAcel, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMaxVelocity(ArmConstants.kMaxVelocity, ArmConstants.kSlotID);
        m_pidController.setSmartMotionAllowedClosedLoopError(ArmConstants.kAllowedError, ArmConstants.kSlotID);
        m_pidController.setSmartMotionMinOutputVelocity(ArmConstants.kMinVelocity, ArmConstants.kSlotID);

        resetEncoder();
    }

    public void periodic() {
        SmartDashboard.putNumber("Arm Position", getPosition());
    }

    /**
     * @return Current arm position (motor rotations)
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
     * @return Whether the arm is at the setpoint
     */
    public boolean atSetpoint() {
        return (Math.abs(m_setPosition - getPosition()) <= ArmConstants.kAllowedError);
    }

    /**
     * @param speed Percent output of the arm
     */
    public void setPercentOutput(double speed) {
        if (speed < 0 && getPosition() < ArmConstants.kMinPosition)
            m_motor.set(0);
        else
            m_motor.set(speed);
    }

    /**
     * @param position Setpoint (motor rotations)
     */
    public void setPosition(double position) {
        m_setPosition = position;
        m_pidController.setReference(position, ControlType.kSmartMotion, ArmConstants.kSlotID);
    }

    /**
     * Zero the encoder position
     */
    public void resetEncoder() {
        m_encoder.setPosition(0);
        setPosition(0);
    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Arm");
        shuffleboardTab.addNumber("Encoder Position", () -> getPosition()).withSize(4, 2).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Encoder Velocity", () -> getVelocity()).withSize(4, 2).withPosition(4, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addBoolean("At setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}