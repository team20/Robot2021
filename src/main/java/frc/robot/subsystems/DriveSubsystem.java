package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ExternalFollower;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ControlType;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.plant.DCMotor;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.VecBuilder;
import frc.robot.Constants.DriveConstants;
import frc.robot.ShuffleboardLogging;

public class DriveSubsystem extends SubsystemBase implements ShuffleboardLogging {

        //talons are the masters
        private final TalonSRX m_masterLeft = new TalonSRX(DriveConstants.kMasterLeftPort);
        private final TalonSRX m_masterRight = new TalonSRX(DriveConstants.kMasterRightPort);
        private final CANSparkMax m_followerLeft = new CANSparkMax(DriveConstants.kFollowerLeftPort,
                        MotorType.kBrushless);
        private final CANSparkMax m_followerRight = new CANSparkMax(DriveConstants.kFollowerRightPort,
                        MotorType.kBrushless);

        private final AHRS m_gyro = new AHRS(DriveConstants.kGyroPort);
        private final DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(
                        Rotation2d.fromDegrees(getHeading()));

        /**
         * Initializes a new instance of the {@link DriveSubsystem} class.
         */
        public DriveSubsystem() {

                m_masterRight.configFactoryDefault();
                m_masterRight.setInverted(DriveConstants.kMasterRightInvert);
                m_masterRight.setNeutralMode(NeutralMode.Brake);
                m_masterRight.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                m_masterRight.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                m_masterRight.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis));
                m_masterRight.configOpenloopRamp(DriveConstants.kRampRate);

                m_masterLeft.configFactoryDefault();
                m_masterLeft.setInverted(DriveConstants.kMasterLeftInvert);
                m_masterLeft.setNeutralMode(NeutralMode.Brake);
                m_masterLeft.configVoltageCompSaturation(DriveConstants.kVoltageComp); 
                m_masterLeft.enableVoltageCompensation(DriveConstants.kEnableVoltageComp);
                m_masterLeft.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,
                DriveConstants.kSmartCurrentLimit, DriveConstants.kPeakCurrentLimit,
                DriveConstants.kPeakCurrentDurationMillis));
                m_masterLeft.configOpenloopRamp(DriveConstants.kRampRate); 
        
                m_followerLeft.restoreFactoryDefaults();
                m_followerLeft.setInverted(DriveConstants.kFollowerLeftOppose);
                m_followerLeft.setIdleMode(IdleMode.kCoast);
                m_followerLeft.enableVoltageCompensation(DriveConstants.kVoltageComp);
                m_followerLeft.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                m_followerLeft.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                                DriveConstants.kPeakCurrentDurationMillis);
                m_followerLeft.setOpenLoopRampRate(DriveConstants.kRampRate);

                m_followerRight.restoreFactoryDefaults();
                m_followerRight.setInverted(DriveConstants.kFollowerRightOppose);
                m_followerRight.setIdleMode(IdleMode.kCoast);
                m_followerRight.enableVoltageCompensation(DriveConstants.kVoltageComp);
                m_followerRight.setSmartCurrentLimit(DriveConstants.kSmartCurrentLimit);
                m_followerRight.setSecondaryCurrentLimit(DriveConstants.kPeakCurrentLimit,
                                DriveConstants.kPeakCurrentDurationMillis);
                m_followerRight.setOpenLoopRampRate(DriveConstants.kRampRate);

                // m_followerLeft.follow(ExternalFollower.kFollowerPhoenix, m_masterLeft.getDeviceID());
                // m_followerRight.follow(ExternalFollower.kFollowerPhoenix, m_masterRight.getDeviceID());

                m_masterLeft.configNominalOutputForward(0);
                m_masterLeft.configNominalOutputReverse(0);
                m_masterLeft.configPeakOutputForward(1);
                m_masterLeft.configPeakOutputReverse(-1);

                m_masterLeft.config_kP(DriveConstants.kSlotID, DriveConstants.kP);
                m_masterLeft.config_kI(DriveConstants.kSlotID, DriveConstants.kI);
                m_masterLeft.config_IntegralZone(DriveConstants.kSlotID, (int) DriveConstants.kIz);
                m_masterLeft.config_kD(DriveConstants.kSlotID, DriveConstants.kD);
                m_masterLeft.config_kF(DriveConstants.kSlotID, DriveConstants.kFF);

                m_masterRight.configNominalOutputForward(0);
                m_masterRight.configNominalOutputReverse(0);
                m_masterRight.configPeakOutputForward(1);
                m_masterRight.configPeakOutputReverse(-1);

                m_masterRight.config_kP(DriveConstants.kSlotID, DriveConstants.kP);
                m_masterRight.config_kI(DriveConstants.kSlotID, DriveConstants.kI);
                m_masterRight.config_IntegralZone(DriveConstants.kSlotID, (int) DriveConstants.kIz);
                m_masterRight.config_kD(DriveConstants.kSlotID, DriveConstants.kD);
                m_masterRight.config_kF(DriveConstants.kSlotID, DriveConstants.kFF);


                //setting up the encoder stuff for the talons
                m_masterLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
                m_masterRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);

                m_masterLeft.setSelectedSensorPosition(0);
                m_masterRight.setSelectedSensorPosition(0);
        
                m_masterLeft.setSensorPhase(DriveConstants.kLeftSensorPhase);
                m_masterRight.setSensorPhase(DriveConstants.kRightSensorPhase);

                resetOdometry(new Pose2d(0, 0, new Rotation2d()));
        }

        /**
         * Update odometry
         */
        public void periodic() {
                SmartDashboard.putNumber("Left wheel", getLeftEncoderPosition());
                SmartDashboard.putNumber("Right wheel", getRightEncoderPosition());
                SmartDashboard.putNumber("Heading",
                m_odometry.getPoseMeters().getRotation().getDegrees());

                m_odometry.update(Rotation2d.fromDegrees(getHeading()),
                getLeftEncoderPosition(), getRightEncoderPosition());

        }

        /**
         * @return The left encoder position (meters)
         */
        public double getLeftEncoderPosition() {
                return m_masterLeft.getSelectedSensorPosition(DriveConstants.kSlotID) * DriveConstants.kEncoderPositionConversionFactor;
                
        }

        /**
         * @return The right encoder position (meters)
         */
        public double getRightEncoderPosition() {
                return m_masterRight.getSelectedSensorPosition(DriveConstants.kSlotID) * DriveConstants.kEncoderPositionConversionFactor; //TODO might need to negate
        }

        /**
         * @return The average encoder distance of both encoders (meters)
         */
        public double getAverageEncoderDistance() {
                return (getRightEncoderPosition() + getLeftEncoderPosition()) / 2.0;
        }

        /**
         * @return The velocity of the left encoder (meters/s)
         */
        public double getLeftEncoderVelocity() {
                return m_masterLeft.getSelectedSensorVelocity(DriveConstants.kSlotID) * DriveConstants.kEncoderVelocityConversionFactor;
        }

        /**
         * @return The velocity of the right encoder (meters/s)
         */
        public double getRightEncoderVelocity() {
                return m_masterRight.getSelectedSensorVelocity(DriveConstants.kSlotID) * DriveConstants.kEncoderVelocityConversionFactor;
                //might need to negative this value
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
                m_masterLeft.setSelectedSensorPosition(0);
                m_masterRight.setSelectedSensorPosition(0);
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
                m_masterLeft.set(ControlMode.PercentOutput, leftSpeed);
                m_masterRight.set(ControlMode.PercentOutput, rightSpeed);
                m_followerLeft.set(m_masterLeft.getMotorOutputPercent());
                m_followerRight.set(m_masterRight.getMotorOutputPercent());

        }

        public void setWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
                m_masterRight.set(ControlMode.Velocity, wheelSpeeds.leftMetersPerSecond);
                m_masterLeft.set(ControlMode.Velocity, wheelSpeeds.rightMetersPerSecond);
                m_followerLeft.set(m_masterLeft.getMotorOutputPercent());
                m_followerRight.set(m_masterRight.getMotorOutputPercent());
               
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