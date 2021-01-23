package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.FlywheelConstants;
import frc.robot.ShuffleboardLogging;

public class FlywheelSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_neoFlywheelMaster = new CANSparkMax(FlywheelConstants.kMasterPort,
            MotorType.kBrushless);
    private final CANSparkMax m_neoFlywheelFollower = new CANSparkMax(FlywheelConstants.kFollowerPort,
            MotorType.kBrushless);
    private final CANPIDController m_neoController = m_neoFlywheelMaster.getPIDController();
    private final CANEncoder m_neoEncoderMaster = m_neoFlywheelMaster.getEncoder();
    private double m_setVelocity;

    /**
     * Initializes a new instance of the {@link FlywheelSubsystem} class.
     */
    public FlywheelSubsystem() {
        // Initialize Motors
        m_neoFlywheelMaster.restoreFactoryDefaults();
        m_neoFlywheelMaster.setInverted(FlywheelConstants.kMasterInvert);
        m_neoFlywheelMaster.setIdleMode(IdleMode.kBrake);
        m_neoFlywheelMaster.enableVoltageCompensation(12);
        m_neoFlywheelMaster.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
        m_neoFlywheelMaster.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
                FlywheelConstants.kPeakCurrentDurationMillis);
        m_neoFlywheelMaster.setSoftLimit(SoftLimitDirection.kForward, 0.0f);

        m_neoFlywheelFollower.restoreFactoryDefaults();
        m_neoFlywheelFollower.setIdleMode(IdleMode.kBrake);
        m_neoFlywheelFollower.enableVoltageCompensation(12);
        m_neoFlywheelFollower.setSmartCurrentLimit(FlywheelConstants.kSmartCurrentLimit);
        m_neoFlywheelFollower.setSecondaryCurrentLimit(FlywheelConstants.kPeakCurrentLimit,
                FlywheelConstants.kPeakCurrentDurationMillis);
        m_neoFlywheelFollower.follow(m_neoFlywheelMaster, FlywheelConstants.kFollowerOppose);

        m_neoEncoderMaster.setPositionConversionFactor(1 / FlywheelConstants.kGearRatio);
        m_neoEncoderMaster.setVelocityConversionFactor(1 / FlywheelConstants.kGearRatio);

        m_neoController.setP(FlywheelConstants.kP);
        m_neoController.setI(FlywheelConstants.kI);
        m_neoController.setD(FlywheelConstants.kD);
        m_neoController.setIZone(FlywheelConstants.kIz);
        m_neoController.setFF(FlywheelConstants.kFF);
        m_neoController.setOutputRange(FlywheelConstants.kMinOutput, FlywheelConstants.kMaxOutput);
    }

    public void periodic() {
        SmartDashboard.putBoolean("Flywheel at Setpoint", atSetpoint());
        SmartDashboard.putNumber("Flywheel Velocity", getVelocity());
        if (m_setVelocity == 0) {
            m_neoFlywheelMaster.stopMotor();
        } else {
            m_neoController.setReference(m_setVelocity, ControlType.kVelocity, 0);
        }

    }

    public void incrementSpeed() {
        setVelocity(m_setVelocity + 50);
    }

    public void decrementSpeed() {
        setVelocity(m_setVelocity - 50);
    }

    /**
     * @return Current setpoint.
     */
    public double getSetpoint() {
        return m_setVelocity;
    }

    /**
     * @return Measured velocity.
     */
    public double getVelocity() {
        return m_neoEncoderMaster.getVelocity();
    }

    /**
     * Sets target speed for flywheel.
     * 
     * @param velocity Target velocity (rpm).
     */
    public void setVelocity(double velocity) {
        m_setVelocity = velocity;
    }

    /**
     * @return Whether the flywheel is at its setpoint ABOVE 0
     */
    public boolean atSetpoint() {
        return getSetpoint() > 0
                ? (Math.abs(getVelocity() - getSetpoint()) / getSetpoint())
                        * 100 < FlywheelConstants.kAllowedErrorPercent
                : false;
    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Flywheel");
        shuffleboardTab.addNumber("Flywheel Velocity", () -> getVelocity()).withSize(4, 2).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addBoolean("At setpoint", () -> atSetpoint()).withSize(1, 1).withPosition(0, 2)
                .withWidget(BuiltInWidgets.kBooleanBox);
        // shuffleboardTab.addNumber("Current draw", () ->
        // m_neoFlywheelMaster.getOutputCurrent() +
        // m_neoFlywheelFollower.getOutputCurrent());
        // shuffleboardTab.addNumber("Setpoint", () ->
        // getSetpoint()).withWidget(BuiltInWidgets.kTextView).withSize(1,
        // 1).withPosition(5, 1);
    }
}
