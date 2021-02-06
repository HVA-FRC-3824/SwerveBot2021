package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.LauncherSetAngle;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Launcher extends SubsystemBase
{
    
    private WPI_TalonFX m_topRightWheel;
    private WPI_TalonFX m_topLeftWheel;
    private WPI_TalonFX m_bottomWheel;
  
    private WPI_TalonSRX m_pivot;
    private AnalogInput m_pivotFeedback;

    public Launcher()
    {
        m_topRightWheel = new WPI_TalonFX(Constants.launcherTopRightMotorID);
        RobotContainer.configureTalonFX(m_topRightWheel, false, false, Constants.launcherTopRightMotorID, Constants.launcherTopRightMotorID,
                                        Constants.launcherTopRightMotorID, Constants.launcherTopRightMotorID);
    
        m_topLeftWheel = new WPI_TalonFX(Constants.launcherTopLeftMotorID);
        RobotContainer.configureTalonFX(m_topLeftWheel, false, false, Constants.launcherTopLeftMotorID, Constants.launcherTopLeftMotorID,
                                        Constants.launcherTopLeftMotorID, Constants.launcherTopLeftMotorID);

        m_bottomWheel = new WPI_TalonFX(Constants.launcherBottomWheelID);
        RobotContainer.configureTalonFX(m_bottomWheel, false, false, Constants.launcherBottomWheelID, Constants.launcherBottomWheelID,
                                        Constants.launcherBottomWheelID, Constants.launcherBottomWheelID);
    
        m_pivot = new WPI_TalonSRX(Constants.launcherPivotID);
        RobotContainer.configureTalonSRX(m_pivot, false, FeedbackDevice.Analog, false, false, 
                                         0.0, 0.0, 0.0, 0.0, 0, 0, false);
    }

   /* Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonFX getTopRightWheelTalonFX()
  {
    return m_topRightWheel;
  }
  public WPI_TalonFX getTopLeftWheelTalonFX()
  {
    return m_topLeftWheel;
  }
  public WPI_TalonFX getBottomWheelTalonFX()
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
    m_topRightWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.launcherTopWheelTPR));
  }
  public void setTopWheelRPM(int rpm)
  {
    m_topRightWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.launcherTopWheelTPR));
  }
  public void setBottomWheelRPM(int rpm)
  {
    m_bottomWheel.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.launcherBottomWheelTPR));
  }

  //Sets power of linear actuator for launcher pivot angle.
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
