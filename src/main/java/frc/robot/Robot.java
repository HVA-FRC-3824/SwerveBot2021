// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// The WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  public static RobotContainer m_robotContainer;

  private Command m_autonomousCommand;
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  // @Overriden

  public void robotInit() {
    m_robotContainer = new RobotContainer();

    CameraServer.getInstance().startAutomaticCapture(0);
  }
  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() 
  {
    CommandScheduler.getInstance().run();
  }

  /**
   * You can set up different autonomous routes through the Java SmartDashboard and LabVIEW Dashboard,
   * all you have to do is select the correct box in either SmartDashboard and LabVIEW to run the route.
   * More routes can be added in the switch case in RobotContainer.
   */
  @Override
  public void autonomousInit() 
  {
    RobotContainer.m_chassis.zeroHeading();
    RobotContainer.m_chassis.resetEncoders();

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    if (m_autonomousCommand != null)
      m_autonomousCommand.schedule();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    if (m_autonomousCommand != null) m_autonomousCommand.cancel();
    RobotContainer.m_limelight.setModeVision();
    RobotContainer.initializeDefaultCommands();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() 
  {
    RobotContainer.m_limelight.turnOffLED();
  }
}
