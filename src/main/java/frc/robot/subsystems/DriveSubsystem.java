package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.ShuffleboardLogging;

public class DriveSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final CANSparkMax m_masterLeft = new CANSparkMax(DriveConstants.kMasterLeftPort, MotorType.kBrushless);
    private final CANSparkMax m_followerLeft = new CANSparkMax(DriveConstants.kFollowerLeftPort, MotorType.kBrushless);
    private final CANSparkMax m_masterRight = new CANSparkMax(DriveConstants.kMasterRightPort, MotorType.kBrushless);
    private final CANSparkMax m_followerRight = new CANSparkMax(DriveConstants.kFollowerRightPort,
            MotorType.kBrushless);
    private final CANEncoder m_leftEncoder = m_masterLeft.getEncoder();
    private final CANEncoder m_rightEncoder = m_masterRight.getEncoder();
    private final CANPIDController m_leftPIDController = m_masterLeft.getPIDController();
    private final CANPIDController m_rightPIDController = m_masterRight.getPIDController();

    private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
    private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
            Rotation2d.fromDegrees(getHeading()));

    /**
     * Initializes a new instance of the {@link DriveSubsystem} class.
     */
    public DriveSubsystem() {
        m_masterLeft.restoreFactoryDefaults();
        m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
        m_masterLeft.setIdleMode(IdleMode.kBrake);
        m_masterLeft.enableVoltageCompensation(12);
        m_masterLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_masterLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis);
        m_masterLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

        m_followerLeft.restoreFactoryDefaults();
        m_followerLeft.setIdleMode(IdleMode.kCoast);
        m_followerLeft.enableVoltageCompensation(12);
        m_followerLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_followerLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis);
        m_followerLeft.setOpenLoopRampRate(DriveConstants.kRampRate);
        m_followerLeft.follow(m_masterLeft, DriveConstants.kFollowerLeftOppose);

        m_masterRight.restoreFactoryDefaults();
        m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
        m_masterRight.setIdleMode(IdleMode.kBrake);
        m_masterRight.enableVoltageCompensation(12);
        m_masterRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_masterRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis);
        m_masterRight.setOpenLoopRampRate(DriveConstants.kRampRate);

        m_followerRight.restoreFactoryDefaults();
        m_followerRight.setIdleMode(IdleMode.kCoast);
        m_followerRight.enableVoltageCompensation(12);
        m_followerRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
        m_followerRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis);
        m_followerRight.setOpenLoopRampRate(DriveConstants.kRampRate);
        m_followerRight.follow(m_masterRight, DriveConstants.kFollowerRightOppose);

        m_leftEncoder.setPositionConversionFactor(
                (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters);
        m_leftEncoder.setVelocityConversionFactor(
                (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters / 60.0);

        m_rightEncoder.setPositionConversionFactor(
                (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters);
        m_rightEncoder.setVelocityConversionFactor(
                (1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters / 60.0);

        m_leftPIDController.setP(DriveConstants.kP);
        m_leftPIDController.setI(DriveConstants.kI);
        m_leftPIDController.setIZone(DriveConstants.kIz);
        m_leftPIDController.setD(DriveConstants.kD);
        m_leftPIDController.setFF(DriveConstants.kFF);
        m_leftPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
        m_leftPIDController.setFeedbackDevice(m_leftEncoder);

        m_rightPIDController.setP(DriveConstants.kP);
        m_rightPIDController.setI(DriveConstants.kI);
        m_rightPIDController.setIZone(DriveConstants.kIz);
        m_rightPIDController.setD(DriveConstants.kD);
        m_rightPIDController.setFF(DriveConstants.kFF);
        m_rightPIDController.setOutputRange(DriveConstants.kMinOutput, DriveConstants.kMaxOutput);
        m_rightPIDController.setFeedbackDevice(m_rightEncoder);

        resetOdometry(new Pose2d(0, 0, new Rotation2d()));
    }

    /**
     * Update odometry
     */
    public void periodic() {
        SmartDashboard.putNumber("Left wheel", getLeftEncoderPosition());
        SmartDashboard.putNumber("Right wheel", getRightEncoderPosition());
        SmartDashboard.putNumber("Heading", m_odometry.getPoseMeters().getRotation().getDegrees());
        m_odometry.update(Rotation2d.fromDegrees(getHeading()), getLeftEncoderPosition(), getRightEncoderPosition());
    }

    /**
     * @return The left encoder position (meters)
     */
    public double getLeftEncoderPosition() {
        return m_leftEncoder.getPosition();
    }

    /**
     * @return The right encoder position (meters)
     */
    public double getRightEncoderPosition() {
        return -m_rightEncoder.getPosition();
    }

    /**
     * @return The average encoder distance of both encoders (meters)
     */
    public double getAverageEncoderDistance() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2.0;
    }

    /**
     * @return The velocity of the left encoder (meters/s)
     */
    public double getLeftEncoderVelocity() {
        return m_leftEncoder.getVelocity();
    }

    /**
     * @return The velocity of the right encoder (meters/s)
     */
    public double getRightEncoderVelocity() {
        return -m_rightEncoder.getVelocity();
    }

    /**
     * @return Pose of the robot
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * @return Wheel speeds of the robot
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getLeftEncoderVelocity(), getRightEncoderVelocity());
    }

    /**
     * @return The heading of the gyro (degrees)
     */
    public double getHeading() {
        return m_gyro.getYaw() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * @return The rate of the gyro turn (deg/s)
     */
    public double getTurnRate() {
        return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    /**
     * Sets both encoders to 0
     */
    public void resetEncoders() {
        m_leftEncoder.setPosition(0);
        m_rightEncoder.setPosition(0);
    }

    /**
     * @param pose Pose to set the robot to
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        m_odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * @param straight Straight percent output
     * @param left     Left percent output
     * @param right    Right percent output
     */
    public void arcadeDrive(double straight, double left, double right) {
        tankDrive(DriveConstants.kSpeedLimitFactor * (straight - left + right),
                DriveConstants.kSpeedLimitFactor * (straight + left - right));
    }

    /**
     * @param leftSpeed  Left motors percent output
     * @param rightSpeed Right motors percent output
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        m_masterLeft.set(leftSpeed);
        m_masterRight.set(rightSpeed);
    }

    public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
        m_leftPIDController.setReference(wheelSpeeds.leftMetersPerSecond, ControlType.kVelocity, DriveConstants.kSlotID,
                DriveConstants.kFeedForward.calculate(wheelSpeeds.leftMetersPerSecond));
        m_rightPIDController.setReference(wheelSpeeds.rightMetersPerSecond, ControlType.kVelocity,
                DriveConstants.kSlotID, DriveConstants.kFeedForward.calculate(wheelSpeeds.rightMetersPerSecond));

    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drive");
        shuffleboardTab.addNumber("Left speed", () -> getWheelSpeeds().leftMetersPerSecond).withSize(4, 2)
                .withPosition(0, 0).withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Right speed", () -> getWheelSpeeds().rightMetersPerSecond).withSize(4, 2)
                .withPosition(4, 0).withWidget(BuiltInWidgets.kGraph);
        shuffleboardTab.addNumber("Left motor speed", () -> getLeftEncoderPosition()).withSize(1, 1)
                .withPosition(0, 2).withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addNumber("Right motor speed", () -> getRightEncoderPosition()).withSize(1, 1)
                .withPosition(1, 2).withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addNumber("Heading", () -> getHeading()).withSize(1, 1).withPosition(2, 2)
                .withWidget(BuiltInWidgets.kTextView);
    }
}