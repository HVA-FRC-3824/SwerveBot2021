package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private WPI_TalonSRX m_wheelIntake;

  public Intake()
  {

    m_wheelIntake = new WPI_TalonSRX(Constants.INTAKE_MOTOR_ID);
    RobotContainer.configureTalonSRX(m_wheelIntake, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, 
                                    Constants.INTAKE_WHEEL_F, Constants.INTAKE_WHEEL_P, 
                                    Constants.INTAKE_WHEEL_I, Constants.INTAKE_WHEEL_D, 0, 0, true);
  }

  /**
   * This method will be called once per scheduler run.
   */
  @Override
  public void periodic()
  {
  }

  /**
   * Methods for Robot.java to get TalonFX/TalonSRX objects to pass to the SetPIDValues command to configure PIDs via SmartDashboard.
   * @return TalonFX/TalonSRX object to be configured.
   */
  public WPI_TalonSRX getWheelIntakeTalonSRX()
  {
      return m_wheelIntake;
  }

  /**
   * Method that spins intake wheels with power.
   * @param power range is from -1.0 to 1.0.
   */
  public void setWheelPower(double power)
  {
    m_wheelIntake.set(ControlMode.PercentOutput, power);
  }

  /**
   * Method sets intake wheel's RPM with ControlMode.Velocity.
   * @param rpm is converted to units per 100 milliseconds for ControlMode.Velocity.
   */
  public void setWheelRPM(int rpm)
  {
    m_wheelIntake.set(ControlMode.Velocity, RobotContainer.convertRPMToVelocity(rpm, Constants.MOTOR_TPR));
  }
}