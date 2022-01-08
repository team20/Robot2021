package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;

//All gear ratios are in the form of driver rotations : driven rotations
public final class Constants {
	public static final class ArduinoConstants {
		public static final int kAddress = 0x1;

		public static final double kAngleP = 0.003;
		public static final double kAngleI = 0.0;
		public static final double kAngleD = 0.0002;
		public static final int kAngleSetpoint = 157;
		// public static final int kAngleTolerance = 10;

		public static final double kDistanceP = 0.01;
		public static final double kDistanceI = 0.0;
		public static final double kDistanceD = 0.0013;
		public static final int kDistanceSetpoint = 20;
		// public static final int kDistanceTolerance = 2;

		public static final int kReadTargetInView = 0;
		public static final int[] kReadXValue = { 1, 2, 3 };
		public static final int[] kReadDistance = { 4, 5, 6 };

		public static final int kWriteMainLEDMode = 0;
		public static final int kWriteMainLEDValue = 1;
		public static final int kWriteShooterLEDMode = 2;
		public static final int kWriteShooterLEDValue = 3;

		public static final class MainLEDModes {
			public static final byte kOff = 0;
			public static final byte kChasing = 1;
		}

		public static final class ShooterLEDModes {
			public static final byte kOff = 0;
			public static final byte kFlywheelPercent = 1;
		}
	}

	public static final class ArmConstants {
		public static final int kMotorPort = 10;
		public static final boolean kInvert = false;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = .0002;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 10_000;
		public static final double kMaxVelocity = 5_000;
		public static final double kAllowedError = 0.1;

		public static final double kOutPosition = -44;
		public static final double kInPosition = -5;
		public static final double kMinPosition = -52;
		public static final double kBounceUpPosition = -40;
		public static final double kBounceDownPosition = -50;
		public static final double kBounceTime = .4;
	}

	public static final class AutoConstants {
		public static final String kInitTrench = "paths/init.path";
	}

	public static final class CarouselConstants {
		public static final int kMotorPort = 14;
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 20;
		public static final double kP = 0.01; // .00001
		public static final double kI = 0;
		public static final double kD = 0.01;
		public static final double kIz = 0;
		public static final double kFF = 0.0138;// 000095;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;

		public static final double kPositionP = 0.0;// TODO - tune
		public static final double kPositionI = 0.0;
		public static final double kPositionD = 0.0;
		public static final double kPositionIz = 0;
		public static final double kPositionFF = 0;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 100;
		public static final double kMaxVelocity = 20;
		public static final double kAllowedError = 0.001;

		public static final int kMagSensorPort = 0;

		public static final double kVelocity = 20;
		public static final double kIntakeVelocity = 30;
		public static final double kJostleVelocity = -65;
		public static final double kGearRatio = 141.0;
		public static final double kStartPositionTolerance = .1; // 5
	}

	public static final class ClimberConstants {
		public static final int kMotorPort = 8; // or 9?
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 40_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.1;

		public static final double kTopSetpoint = -1;
	}

	public static final class ControllerConstants {
		public static final int kDriverControllerPort = 0;
		public static final int kOperatorControllerPort = 1;
		public static final double kDeadzone = 0.1;
		public static final double kTriggerDeadzone = .05;

		public static final class Axis {
			public static final int kLeftX = 0;
			public static final int kLeftY = 1;
			public static final int kRightX = 2;
			public static final int kLeftTrigger = 3;
			public static final int kRightTrigger = 4;
			public static final int kRightY = 5;
		}

		public static final class Button {
			public static final int kSquare = 1;
			public static final int kX = 2;
			public static final int kCircle = 3;
			public static final int kTriangle = 4;
			public static final int kLeftBumper = 5;
			public static final int kRightBumper = 6;
			public static final int kShare = 9;
			public static final int kOptions = 10;
			public static final int kLeftStick = 11;
			public static final int kRightStick = 12;
			public static final int kPS = 13;
			public static final int kTrackpad = 14;
		}

		public static final class DPad {
			public static final int kUp = 0;
			public static final int kRight = 90;
			public static final int kDown = 180;
			public static final int kLeft = 270;
		}
	}

	public static final class DriveConstants {
		public static final int kMasterLeftPort = 4;
		public static final boolean kMasterLeftInvert = false;
		public static final int kFollowerLeftPort = 3;
		public static final boolean kFollowerLeftOppose = false;

		public static final int kMasterRightPort = 5;
		public static final boolean kMasterRightInvert = false;
		public static final int kFollowerRightPort = 6;
		public static final boolean kFollowerRightOppose = false;

		public static final int kSmartCurrentLimit = 60;
		public static final double kPeakCurrentLimit = 75;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kP = .14;//0.198;
		public static final double kI = 0;
		public static final double kD = 0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;

		public static final SPI.Port kGyroPort = SPI.Port.kMXP;
		public static final boolean kGyroReversed = true;

		public static final double ksVolts = 0.196;
		public static final double kvVoltSecondsPerMeter = 2.15;
		public static final double kaVoltSecondsSquaredPerMeter = .53;
		public static final double kTrackwidthMeters = 0.7815245428457417;
		public static final double kMaxSpeedMetersPerSecond = 1;
		public static final double kMaxAccelerationMetersPerSecondSquared = .5;
		public static final double kMaxRotSpeedMetersPerSecond = 1;
		public static final double kWheelDiameterMeters = Units.inchesToMeters(6);
		public static final double kGearRatio = 8.18;

		public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
				kTrackwidthMeters);
		public static final SimpleMotorFeedforward kFeedForward = new SimpleMotorFeedforward(DriveConstants.ksVolts,
				DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
		public static final DifferentialDriveVoltageConstraint kVoltageConstraint = new DifferentialDriveVoltageConstraint(
				DriveConstants.kFeedForward, DriveConstants.kDriveKinematics, 10);
		public static final TrajectoryConfig kTrajectoryConfig = new TrajectoryConfig(
				DriveConstants.kMaxSpeedMetersPerSecond, DriveConstants.kMaxAccelerationMetersPerSecondSquared)
						.setKinematics(DriveConstants.kDriveKinematics)
						.addConstraint(DriveConstants.kVoltageConstraint);

		public static final double kTurningMultiplier = .45;
		public static final double kQuickStopThreshold = .2;
		public static final double kQuickStopAlpha = .1;
		public static final double kBackupDistance = Units.feetToMeters(2);
		public static final double kRampRate = .1;
		public static final double kSpeedLimitFactor = .75;

		public static final boolean kLeftSensorPhase = true; //TODO these are totally arbitrary right now and need to be checked
		public static final boolean kRightSensorPhase = false;

		public static final boolean kEnableVoltageComp = true;
		public static final double kVoltageComp = 12;
		public static final double kEncoderCounts = 4096;

		public static final double kEncoderPositionConversionFactor = (1/DriveConstants.kEncoderCounts)*
		(1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters;

		public static final double kEncoderVelocityConversionFactor = (1/DriveConstants.kEncoderCounts)*
		(1 / DriveConstants.kGearRatio) * Math.PI * DriveConstants.kWheelDiameterMeters * 1000;

	}

	public static final class FeederConstants {
		public static final boolean kInvert = false;
		public static final int kMotorPort = 1;
		public static final double kSpeed = .9;
	}

	public static final class FlywheelConstants {
		public static final int kMasterPort = 11;
		public static final int kFollowerPort = 13;
		public static final boolean kMasterInvert = false;
		public static final boolean kFollowerOppose = true;
		public static final int kSmartCurrentLimit = 50;
		public static final double kPeakCurrentLimit = 60;
		public static final int kPeakCurrentDurationMillis = 100;
		public static final double kP = 0.000_167; // 0.000_375; then .0004
		public static final double kI = 0;
		public static final double kD = 0.000_0125;// 0.000_03;
		public static final double kIz = 0.0;
		public static final double kFF = .000_0804;// 0.000_193;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final double kGearRatio = 1 / 2.4;
		public static final double kAllowedErrorPercent = 5;
	}

	public static final class HoodConstants {
		public static final int kMotorPort = 12;
		public static final boolean kInvert = true;
		public static final int kSmartCurrentLimit = 60;
		public static final double kP = 0.000_1;
		public static final double kI = 0.0;
		public static final double kD = 0.0;
		public static final double kIz = 0;
		public static final double kFF = 0;
		public static final double kMaxOutput = 1;
		public static final double kMinOutput = -1;
		public static final int kSlotID = 0;
		public static final double kMinVelocity = 0;
		public static final double kMaxAcel = 20_000;
		public static final double kMaxVelocity = 10_000;
		public static final double kAllowedError = 0.2;

		public static final double kMinEncoderValue = 0.0;
		public static final double kMaxEncoderValue = 42.0;
		public static final double kMinAngle = 24.36;
		public static final double kMaxAngle = 77.64;
	}

	public static final class IntakeConstants {
		public static final boolean kInvert = true;
		public static final int kMotorPort = 2;
		public static final double kPercentOutput = 1;
	}

	public static final class LimelightConstants {// TODO - Update PID and camera values
		public static final double kDisP = .016;
		public static final double kDisI = 0;
		public static final double kDisD = 0;
		public static final double kTurnP = 0.2;
		public static final double kTurnI = 0.0000;
		public static final double kTurnD = 0.01;
		public static final double kTurnTolerance = .04;
		public static final double kDistanceTolerance = .1;
		public static final double kCameraHeight = 27.6;
		public static final double kCameraAngle = 18.43;
		public static final double kTargetHeight = 89.75;
	}

	public static final class LoggingConstants {
		// Arduino, Arm, Carousel, Climber, Drive, Feeder, Flywheel, Hood, Intake,
		// Limelight
		public static final boolean[] kSubsystems = { false, false, true, false, true, false, false, false, false,
				false };
	}

	public enum FieldLocation {
		WALL(2700, 0, 50, 0, 0), TWOFEET(2850, 7, 25, 0, 0), INITLINE(3500, 23, 30, 0, 0),
		CLOSETRENCH(/* 5500 */4700, /* 37.4 */33.5, 20, 0, 0), FARTWRENCH(6500, 40, 20, 0, 0);

		public final double flywheelSetpoint, hoodSetpoint, carouselSetpoint, distanceGoal, turnGoal;

		private FieldLocation(double flywheelSetpoint, double hoodSetpoint, double carouselSetpoint,
				double distanceGoal, double turnGoal) {
			this.flywheelSetpoint = flywheelSetpoint;
			this.hoodSetpoint = hoodSetpoint;
			this.carouselSetpoint = carouselSetpoint;
			this.distanceGoal = distanceGoal;
			this.turnGoal = turnGoal;
		}

		public static final FieldLocation fromDistance(double distance) {
			FieldLocation closestDistance = WALL;
			for (FieldLocation fieldLocation : FieldLocation.values()) {
				if (Math.abs(distance - fieldLocation.distanceGoal) < Math
						.abs(distance - closestDistance.distanceGoal)) {
					closestDistance = fieldLocation;
				}
			}
			return closestDistance;
		}

		public static final FieldLocation fromFlywheelSetpoint(double flywheelSetpoint) {
			FieldLocation closestSetpoint = WALL;
			for (FieldLocation fieldLocation : FieldLocation.values()) {
				if (Math.abs(flywheelSetpoint - fieldLocation.flywheelSetpoint) < Math
						.abs(flywheelSetpoint - closestSetpoint.flywheelSetpoint)) {
					closestSetpoint = fieldLocation;
				}
			}
			return closestSetpoint;
		}
	}
}