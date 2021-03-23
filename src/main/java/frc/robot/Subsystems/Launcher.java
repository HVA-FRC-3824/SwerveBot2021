package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase
{
    
    private WPI_TalonSRX m_topRightWheel;
    private WPI_TalonSRX m_topLeftWheel;
    private WPI_TalonSRX m_bottomWheel;

    public Launcher()
    {

      m_topRightWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_RIGHT_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_topRightWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                      Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, false);

      m_topLeftWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_LEFT_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_topLeftWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                      Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, false);

      m_bottomWheel = new WPI_TalonSRX(Constants.LAUNCHER_BOTTOM_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_bottomWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, false, false, 
                                      Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, false);

    }

   /* Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getTopRightWheelTalonSRX()
  {
    return m_topRightWheel;
  }
  public WPI_TalonSRX getTopLeftWheelTalonSRX()
  {
    return m_topLeftWheel;
  }
  public WPI_TalonSRX getBottomWheelTalonSRX()
  {
    return m_bottomWheel;
  }

  /** Sets the power output of the top and bottom launcher wheels.
  * @param power is between -1.0 and 1.0.
  */
  public void setTopRightWheelPower(double power)
  {
    m_topRightWheel.set(ControlMode.PercentOutput, power);
  }
  public void setTopLeftWheelPower(double power)
  {
    m_topLeftWheel.set(ControlMode.PercentOutput, power);
  }
  public void setBottomWheelPower(double power)
  {
    m_bottomWheel.set(ControlMode.PercentOutput, power);
  }

  /** Sets the RPM of the top and bottom launcher wheels.
  * @param rpm is converted to a velocity (units/100ms) for the launcher wheels PID to be set to.
  */
  public void setTopLeftWheelRPM(int rpm)
  {
    m_topLeftWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }
  public void setTopRightWheelRPM(int rpm)
  {
    m_topRightWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }
  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }

  public void stopLauncher()
  {
    this.setTopLeftWheelPower(0.0);
    this.setTopRightWheelPower(0.0);
    this.setBottomWheelPower(0.0);
  }
}
