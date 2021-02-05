package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class Chassis extends SubsystemBase
{

    //region variables

    private AHRS m_ahrs;

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
    
        //Try to instantiate the navX gyro with exception
        try
        {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } 
        catch (RuntimeException ex) 
        {
            System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
        }
    }

    public void convertSwerveValues (double x1, double y1, double x2)
    {
        //Width and length of robot
        double w = 25;
        double l = 17;

        //Width and length relative ratios
        double wR;
        double lR;
        
        //Input velocities and turn
        double vX   = 0;
        double vY   = 0;
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
        wR = Math.cos(turnAngle);
        lR = Math.sin(turnAngle);

        //Apply dead zone for velocities
        if(Math.abs(x1) > 0.15) vX = x1;
        if(Math.abs(y1) > 0.15) vY = -y1;

        //Establishing swerve gyro difference
        double gyroCurrent = m_ahrs.getYaw();

        //Adjust strafe vector so that forward constant
        double r = Math.sqrt(vX * vX + vY * vY);
        double strafeAngle = Math.atan2(vY, vX);

        strafeAngle += gyroCurrent / 360 * 2 * Math.PI;
        vX = r * Math.cos(strafeAngle);
        vY = r * Math.sin(strafeAngle);

        //Shortening equations for adding strafe and turn for each wheel
        a = vX - turn * lR;
        b = vX + turn * lR;
        c = vY - turn * wR;
        d = vY + turn * wR;

        //Adjust for exceeding max speed
        double highestSpeed = Math.max(Math.max(Math.max(frontRight[0], frontLeft[0]), backLeft[0]), backRight[0]);


        //Finding speed of each wheel based on x and y velocities
 
        if(highestSpeed > 1){
            frontRight[0] = frontRight[0] / highestSpeed;
            frontLeft[0] = frontLeft[0] / highestSpeed;
            backLeft[0] = backLeft[0] / highestSpeed;
            backRight[0] = backRight[0] / highestSpeed;
        }

        //update last angle
        frontRight[4] = frontRight[3];
        frontLeft[4] = frontLeft[3];
        backLeft[4] = backLeft[3];
        backRight[4] = backLeft[3];

        //Set new angles
        if(!(vX == 0 && vY == 0 && turn == 0)){
            //Find angle of each wheel based on velocities
            frontRight[3] = Math.atan2(c, b) - Math.PI/2;
            frontLeft[3] = Math.atan2(d, b) - Math.PI/2;
            backLeft[3] = Math.atan2(d, a) - Math.PI/2;
            backRight[3] = Math.atan2(c, a) - Math.PI/2;
        }

        //When a wheel moves more than half a circle in one direction, offsets so it goes the shorter route
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] -= 2 * Math.PI;
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] += 2 * Math.PI;
        // if((Math.abs(frontL[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] -= 2 * Math.PI;
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] += 2 * Math.PI;

        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] -= 2 * Math.PI;
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] += 2 * Math.PI;
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] -= 2 * Math.PI;
        if((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1])) frontRight[3] += 2 * Math.PI;
        
        

    }
     
    //endregion

} 