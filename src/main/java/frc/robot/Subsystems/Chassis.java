package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Chassis extends SubsystemBase
{

    //region variables

    //region motors
    private WPI_TalonFX m_angleMotorFrontRight;
    private WPI_TalonFX m_speedMotorFrontRight;
    
    private WPI_TalonFX m_angleMotorFrontLeft;
    private WPI_TalonFX m_speedMotorFrontLeft;

    private WPI_TalonFX m_angleMotorBackLeft;
    private WPI_TalonFX m_speedMotorBackLeft;
 
    private WPI_TalonFX m_angleMotorBackRight;
    private WPI_TalonFX m_speedMotorBackRight;

    //[Speed, Angle, Previous Angle, Offset]
    private double [] frontRight = {0, 0, 0, 0};
    private double [] frontLeft = {0, 0, 0, 0};
    private double [] backLeft = {0, 0, 0, 0};
    private double [] backRight = {0, 0, 0, 0};

    //endregion

    //endregion

    
    //region methods
    
    public Chassis ()
    {
        
        //Instantiating Drivetrain objects

        m_angleMotorFrontRight = new WPI_TalonFX(Constants.frontRightAngleID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
        
        m_speedMotorFrontRight = new WPI_TalonFX(Constants.frontRightSpeedID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
                
        m_angleMotorFrontLeft = new WPI_TalonFX(Constants.frontLeftAngleID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
                
        m_speedMotorFrontLeft = new WPI_TalonFX(Constants.frontLeftSpeedID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
        
        m_angleMotorBackLeft = new WPI_TalonFX(Constants.backLeftAngleID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
        
        m_speedMotorBackLeft = new WPI_TalonFX(Constants.backLeftSpeedID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
                
        m_angleMotorBackRight = new WPI_TalonFX(Constants.backRightAngleID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
                
        m_speedMotorBackRight = new WPI_TalonFX(Constants.backRightSpeedID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);
    
    }

    public void convertSwerveValues (double x1, double y1, double x2)
    {
        //Width and length of robot
        double w = 25;
        double l = 17;

        //Width and length relative ratios
        double wr;
        double lr;
        
        //Input velocities and turn
        double VX   = 0;
        double VY   = 0;
        double turn = 0;

        //
        double a;
        double b;
        double c;
        double d;
        
        //Apply deadzone to turn analog stick
        if (Math.abs(x2) > 0.15) turn = x2;

        //Find similar triangles to chassis for turn vectors (radius = 1)
        double turnAngle = Math.atan2(l, w);
        wr = Math.cos(turnAngle);
        lr = Math.sin(turnAngle);

        //Apply dead zone for velocities
        if(Math.abs(x1) > 0.15) VX = x1;
        
        

    }
     
    //endregion

} 