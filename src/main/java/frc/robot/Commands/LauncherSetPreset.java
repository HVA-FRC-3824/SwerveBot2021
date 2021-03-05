package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LauncherSetPreset extends CommandBase 
{
  public LauncherSetPreset()
  {
    addRequirements(RobotContainer.m_launcher);
  }

  @Override
  public void initialize()
  {
    /* Based on the position of the switch on operator board, set preset of launcher. */
    if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_GREEN_BTN_ID))
    {
      RobotContainer.m_launcher.setTopLeftWheelRPM(Constants.LAUNCHER_GREEN_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setTopRightWheelRPM(Constants.LAUNCHER_GREEN_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setBottomWheelRPM(Constants.LAUNCHER_GREEN_ZONE_MOTOR_RPM);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_YELLOW_BTN_ID))
    {
      RobotContainer.m_launcher.setTopLeftWheelRPM(Constants.LAUNCHER_YELLOW_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setTopRightWheelRPM(Constants.LAUNCHER_YELLOW_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setBottomWheelRPM(Constants.LAUNCHER_YELLOW_ZONE_MOTOR_RPM);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_BLUE_BTN_ID))
    {
      RobotContainer.m_launcher.setTopLeftWheelRPM(Constants.LAUNCHER_BLUE_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setTopRightWheelRPM(Constants.LAUNCHER_BLUE_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setBottomWheelRPM(Constants.LAUNCHER_BLUE_ZONE_MOTOR_RPM);
    }
    else if (RobotContainer.m_OI.getOperatorController().getRawButton(Constants.OPERATOR_LAUNCHER_PRESET_RED_BTN_ID))
    {
      RobotContainer.m_launcher.setTopLeftWheelRPM(Constants.LAUNCHER_RED_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setTopRightWheelRPM(Constants.LAUNCHER_RED_ZONE_MOTOR_RPM);
      RobotContainer.m_launcher.setBottomWheelRPM(Constants.LAUNCHER_RED_ZONE_MOTOR_RPM);
    }
  }
  
  @Override
  public void execute()
  {
  }

  @Override
  public void end(boolean interrupted)
  {
  }

  @Override
  public boolean isFinished()
  {
    return true;
  }
}