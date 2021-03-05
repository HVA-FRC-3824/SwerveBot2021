
package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Commands.ChamberIndexBalls;
import edu.wpi.first.wpilibj.Ultrasonic;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Chamber extends SubsystemBase
{

    private WPI_TalonSRX m_elevator;
    
    private Ultrasonic m_ballPos_entering;
    private Ultrasonic m_ballPos_exiting;

    public Chamber()
    {
        m_elevator = new WPI_TalonSRX(Constants.CHAMBER_WHEEL_ID);
        RobotContainer.configureTalonSRX(m_elevator, false, FeedbackDevice.CTRE_MagEncoder_Relative, true, true, 
                                        Constants.CHAMBER_F, Constants.CHAMBER_P, Constants.CHAMBER_I, Constants.CHAMBER_D, 0, 0, true);
        
        m_ballPos_entering = new Ultrasonic(Constants.CHAMBER_BALL_POS_ENTER_PING, Constants.CHAMBER_BALL_POS_ENTER_ECHO);
        m_ballPos_exiting = new Ultrasonic(Constants.CHAMBER_BALL_POS_EXIT_PING, Constants.CHAMBER_BALL_POS_EXIT_ECHO);
    }

    public WPI_TalonSRX getElevatorTalonSRX()
    {
        return m_elevator;
    }

      /**
     * Methods to get ultrasonic readings to track if a ball is present or not.
     * @return the distance the ultrasonic is reading.
     */
    public double getEnteringRange()
    {
        return m_ballPos_entering.getRangeInches();
    }
    public double getExitingRange()
    {
        return m_ballPos_exiting.getRangeInches();
    }

    public void initAutoIndex()
    {
        Command autoIndexBalls = new ChamberIndexBalls();
        autoIndexBalls.schedule();
    }

    public void runChamber(double power)
    {
        m_elevator.set(ControlMode.PercentOutput, power);
    }

    public void setElevatorPower (double power)
    {
        m_elevator.set(ControlMode.PercentOutput, power);
    }
    
    public void startUltrasonics()
    {
        m_ballPos_entering.setAutomaticMode(true);
        m_ballPos_exiting.setAutomaticMode(true);
    }

      /**
     * Method to move the chamber the specified distance from its current position.
     */
    public void stepChamberDistance(double distance)
    {
        double presentPosition = m_elevator.getSelectedSensorPosition();
        m_elevator.set(ControlMode.MotionMagic, presentPosition + distance);
    }
}