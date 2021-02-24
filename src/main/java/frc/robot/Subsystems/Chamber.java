
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.WPI_AutoFeedEnable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Chamber extends SubsystemBase
{

    private WPI_TalonSRX m_wheelMotor;
    
    private DoubleSolenoid m_output;

    private Ultrasonic m_ballPos_entering;
    private Ultrasonic m_ballPos_exiting;

    public Chamber()
    {
        m_wheelMotor = new WPI_TalonSRX(Constants.CHAMBER_WHEEL_ID);
        RobotContainer.configureTalonSRX(m_wheelMotor, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, 
                                        Constants.CHAMBER_F, Constants.CHAMBER_P, Constants.CHAMBER_I, Constants.CHAMBER_D, 0, 0, true);

        m_output = new DoubleSolenoid(Constants.CHAMBER_OUTPUT_PORT_A, Constants.CHAMBER_OUTPUT_PORT_B);
    
    }

    public void startUltrasonics()
    {
        m_ballPos_entering.setAutomaticMode(true);
        m_ballPos_exiting.setAutomaticMode(true);
    }

    public void runChamber(double power)
    {
        m_wheelMotor.set(ControlMode.PercentOutput, power);
    }
}