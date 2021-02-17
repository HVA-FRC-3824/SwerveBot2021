
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.WPI_AutoFeedEnable;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Chamber extends SubsystemBase
{

    private WPI_TalonSRX m_drive;
    
    private DoubleSolenoid m_extender;

    private Ultrasonic m_ballPos_entering;
    private Ultrasonic m_ballPos_exiting;

    public Chamber()
    {
        m_drive = new WPI_TalonSRX(Constants.chamberDriveID);
        RobotContainer.configureTalonSRX(m_drive, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, Constants.launcherF, Constants.launcherP, Constants.launcherI, Constants.launcherD, 0, 0, true);

        m_extender = new DoubleSolenoid(Constants.chamberExtenderPortA, Constants.chamberExtenderPortB);
    
    }

}