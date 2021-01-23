package frc.robot.subsystems;

import java.util.ArrayList;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;
import frc.robot.ShuffleboardLogging;

public class LimelightSubsystem extends SubsystemBase implements ShuffleboardLogging {

    private final NetworkTable m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    private boolean isTargetVisible;
    private double xAngle, yAngle, distance;
    private ArrayList<Double> averageDistance = new ArrayList<>();

    public LimelightSubsystem() {
        turnOffLight();
    }

    /**
     * Update local data from the limelight network table
     */
    public void periodic() {
        isTargetVisible = m_limelightTable.getEntry("tv").getDouble(0) == 1;
        xAngle = m_limelightTable.getEntry("tx").getDouble(0);
        yAngle = m_limelightTable.getEntry("ty").getDouble(0);
        distance = isTargetVisible
                ? (LimelightConstants.kTargetHeight - LimelightConstants.kCameraHeight)
                        / (Math.tan(Math.toRadians(LimelightConstants.kCameraAngle + getYAngle())))
                : 0;
        if (distance != 0) {
            averageDistance.add(distance);
            if (averageDistance.size() > 10) {
                averageDistance.remove(0);
            }
        }

        SmartDashboard.putNumber("Avg Distance", getAverageDistance());
        SmartDashboard.putNumber("Distance", getDistance());
        SmartDashboard.putBoolean("Target Visible", isTargetVisible());
    }

    /**
     * @return Whether the limelight can see the target
     */
    public boolean isTargetVisible() {
        return isTargetVisible;
    }

    /**
     * @return The x angle of the target center to the limelight crosshair
     */
    public double getXAngle() {
        return xAngle;
    }

    /**
     * @return The y angle of the target center to the limelight crosshair
     */
    public double getYAngle() {
        return yAngle;
    }

    /**
     * @return The estimated ground distance from the limelight to the target
     */
    public double getDistance() {
        return distance;
    }

    /**
     * @return The averaged ground distance from the limelight to the target over
     *         the past .2 seconds (account for loss in camera view while
     *         movingfast)
     */
    public double getAverageDistance() {
        double sum = 0;
        double size = averageDistance.size();
        for (int i = 0; i < averageDistance.size(); i++) {
            sum += averageDistance.get(i);
        }
        return size > 0 ? sum / size : 0;
    }

    /**
     * @return Whether the LIME light is on
     */
    public boolean isLightOn() {
        return m_limelightTable.getEntry("ledmode").getDouble(0) == 1;
    }

    /**
     * Turn on the LIME light
     */
    public void turnOnLight() {
        m_limelightTable.getEntry("ledmode").setNumber(1);
    }

    /**
     * Turn off the LIME light
     */
    public void turnOffLight() {
        m_limelightTable.getEntry("ledmode").setNumber(0);
    }

    public void configureShuffleboard() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Limelight");
        shuffleboardTab.addNumber("X Angle", () -> getXAngle()).withSize(1, 1).withPosition(0, 0)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addNumber("Distance", () -> getDistance()).withSize(1, 1).withPosition(1, 0)
                .withWidget(BuiltInWidgets.kTextView);
        shuffleboardTab.addBoolean("Target Visible", () -> isTargetVisible()).withSize(1, 1).withPosition(0, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
        shuffleboardTab.addBoolean("Light On", () -> isLightOn()).withSize(1, 1).withPosition(1, 1)
                .withWidget(BuiltInWidgets.kBooleanBox);
    }
}
