package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.LauncherSetAngle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase
{
    
    private WPI_TalonSRX m_topRightWheel;
    private WPI_TalonSRX m_topLeftWheel;
    private WPI_TalonSRX m_bottomWheel;
  
    private WPI_TalonSRX m_pivot;
    private AnalogInput m_pivotFeedback;

    public Launcher()
    {
      
      // Region Instantiate motors

      m_topRightWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_RIGHT_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_topRightWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, true);

      m_topLeftWheel = new WPI_TalonSRX(Constants.LAUNCHER_TOP_LEFT_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_topLeftWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, true);

      m_bottomWheel = new WPI_TalonSRX(Constants.LAUNCHER_BOTTOM_MOTOR_ID);
      RobotContainer.configureTalonSRX(m_bottomWheel, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, Constants.LAUNCHER_F, Constants.LAUNCHER_P, Constants.LAUNCHER_I, Constants.LAUNCHER_D, 0, 0, true);
      
      // Endregion

    }

   /* Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getTopRightWheelTalonFX()
  {
    return m_topRightWheel;
  }
  public WPI_TalonSRX getTopLeftWheelTalonFX()
  {
    return m_topLeftWheel;
  }
  public WPI_TalonSRX getBottomWheelTalonFX()
  {
    return m_bottomWheel;
  }
  public WPI_TalonSRX getPivotTalonSRX()
  {
    return m_pivot;
  }

   /* Sets the power output of the top and bottom launcher wheels.
   * @param power is between -1.0 and 1.0.*/
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

   /* Sets the RPM of the top and bottom launcher wheels.
   * @param rpm is converted to a velocity (units/100ms) for the launcher wheels PID to be set to.*/
  public void setTopLeftWheelRPM(int rpm)
  {
    m_topLeftWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }
  public void setTopWheelRPM(int rpm)
  {
    m_topRightWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }
  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }

  // Sets power of linear actuator for launcher pivot angle.
  public void setPivotPower(double power)
  {
    if ((this.getPivotADC() <= 2000 && power <= 0.0) || (this.getPivotADC() >= 3500 && power >= 0.0))
    {
      m_pivot.set(ControlMode.PercentOutput, 0.0);
    }
    else
    {
      m_pivot.set(ControlMode.PercentOutput, -power);
    }
  }

   /* Sets the desired position of the launcher pivot angle.
   * @param position will be used as the setpoint for the PID controller*/
  public void setAngle(int setpoint)
  {
    Command setAngle = new LauncherSetAngle(setpoint);
    setAngle.schedule();
  }

  /**
   * Get current ADC of pivot linear actuator to verify it is not overdriven.
   */
  public int getPivotADC()
  {
    return m_pivotFeedback.getValue();
  }

}
