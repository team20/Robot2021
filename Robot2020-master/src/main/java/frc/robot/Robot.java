package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

	private RobotContainer m_robotContainer;
	private Command m_autonomousCommand = null;

	public void robotInit() {
		m_robotContainer = new RobotContainer();
	}

	public void robotPeriodic() {
		CommandScheduler.getInstance().run();
	}

	public void disabledInit() {

	}

	public void disabledPeriodic() {

	}

	public void autonomousInit() {
		m_autonomousCommand = m_robotContainer.getAutonomousCommand();
		if (m_autonomousCommand != null) {
			m_autonomousCommand.schedule();
		}
	}

	public void autonomousPeriodic() {

	}

	public void teleopInit() {
		if (m_autonomousCommand != null) {
			m_autonomousCommand.cancel();
		}
	}

	public void teleopPeriodic() {

	}

	public void testInit() {

	}

	public void testPeriodic() {

	}
}