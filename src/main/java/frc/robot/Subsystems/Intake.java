package frc.robot.Subsystems;

import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase
{
  private DoubleSolenoid m_leftExtender;
  private DoubleSolenoid m_rightExtender;

  private WPI_TalonSRX m_wheelIntake;

  public Intake()
  {
    
    m_leftExtender = new DoubleSolenoid(Constants.INTAKE_LEFT_EXTENDER_PORT_A, Constants.INTAKE_LEFT_EXTENDER_PORT_B);
    m_rightExtender = new DoubleSolenoid(Constants.INTAKE_RIGHT_EXTENDER_PORT_A, Constants.INTAKE_RIGHT_EXTENDER_PORT_B);

    m_wheelIntake = new WPI_TalonSRX(Constants.INTAKE_MOTOR_ID);
    RobotContainer.configureTalonSRX(m_wheelIntake, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, 
                                    Constants.INTAKE_WHEEL_F, Constants.INTAKE_WHEEL_P, 
                                    Constants.INTAKE_WHEEL_I, Constants.INTAKE_WHEEL_D, 0, 0, true);

    SmartDashboard.putData("RETRACT", new InstantCommand(() -> this.retractExtender()));
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
   * Gets the opposite value of the current solenoid value for toggling extender.
   * @return the solenoid value the extender should be set to in order to toggle.
   */
  private Value getLeftExtenderValueToToggle()
  {
    if (m_leftExtender.get() == Value.kForward)
      return Value.kReverse;
    else
      return Value.kForward;
  }

  private Value getRightExtenderValueToToggle()
  {
    if (m_rightExtender.get() == Value.kForward)
      return Value.kReverse;
    else 
      return Value.kForward;
  }

  /**
   * Method that toggles the intake pistons between being retracted and extended.
   */
  public void toggleExtender()
  {
    m_leftExtender.set(this.getLeftExtenderValueToToggle());
    m_rightExtender.set(this.getRightExtenderValueToToggle());
  }

  /**
   * Methods to extend/retract intake pistons.
   */
  public void extendExtender()
  {
    m_leftExtender.set(Value.kReverse);
    m_rightExtender.set(Value.kReverse);
  }
  public void retractExtender()
  {
    m_leftExtender.set(Value.kForward);
    m_rightExtender.set(Value.kForward);
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