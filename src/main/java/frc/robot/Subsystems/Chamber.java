
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.WPI_AutoFeedEnable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Chamber extends SubsystemBase
{

    private WPI_TalonSRX m_base;
    
    private Ultrasonic m_ballPos_entering;
    private Ultrasonic m_ballPos_exiting;

    public Chamber()
    {

    }

}