package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;

public class Chassis extends SubsystemBase {

    // Region Variables

    public AHRS m_ahrs;

    // Region Motors
    private WPI_TalonFX m_angleMotorFrontRight;
    private WPI_TalonFX m_speedMotorFrontRight;

    private WPI_TalonFX m_angleMotorFrontLeft;
    private WPI_TalonFX m_speedMotorFrontLeft;

    private WPI_TalonFX m_angleMotorBackLeft;
    private WPI_TalonFX m_speedMotorBackLeft;

    private WPI_TalonFX m_angleMotorBackRight;
    private WPI_TalonFX m_speedMotorBackRight;

    private SwerveModuleState m_frontRightState;
    private SwerveModuleState m_frontLeftState;
    private SwerveModuleState m_backLeftState;
    private SwerveModuleState m_backRightState;
    private SwerveModuleState[] moduleStates;

    private SwerveDriveKinematics m_swerveDriveKinematics;
    public  SwerveDriveOdometry m_swerveDriveOdometry;

    private PIDController m_xController;
    private PIDController m_yController;
    private ProfiledPIDController m_angleController;

    // [Speed, Angle, Previous Angle, Offset]
    private double[] frontRight = { 0, 0, 0, 0 };
    private double[] frontLeft = { 0, 0, 0, 0 };
    private double[] backLeft = { 0, 0, 0, 0 };
    private double[] backRight = { 0, 0, 0, 0 };

    // Endregion

    // Endregion

// Main Method
    public Chassis() 
    {

        // Instantiating Drivetrain objects

        m_angleMotorFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_ANGLE_ID);
                                    RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 
                                    Constants.K_CHASSIS_ANGLE_P, Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

        m_speedMotorFrontRight = new WPI_TalonFX(Constants.FRONT_RIGHT_SPEED_ID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);

        m_angleMotorFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_ANGLE_ID);
                                    RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0,
                                    Constants.K_CHASSIS_ANGLE_P, Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

        m_speedMotorFrontLeft = new WPI_TalonFX(Constants.FRONT_LEFT_SPEED_ID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);

        m_angleMotorBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_ANGLE_ID);
                                    RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 
                                    Constants.K_CHASSIS_ANGLE_P, Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

        m_speedMotorBackLeft = new WPI_TalonFX(Constants.BACK_LEFT_SPEED_ID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);

        m_angleMotorBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_ANGLE_ID);     
                                    RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 
                                    Constants.K_CHASSIS_ANGLE_P, Constants.K_CHASSIS_ANGLE_I, Constants.K_CHASSIS_ANGLE_D);

        m_speedMotorBackRight = new WPI_TalonFX(Constants.BACK_RIGHT_SPEED_ID);
        RobotContainer.configureTalonFX(m_angleMotorFrontRight, false, false, 0.0, 0.0, 0.0, 0.0);

        // Try to instantiate the navX gyro with exception
        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
        }

        // Instantiating the Swerve Kinematics & Odometry
        m_swerveDriveKinematics = new SwerveDriveKinematics(Constants.FRONT_RIGHT_LOCATION_M, Constants.FRONT_LEFT_LOCATOIN_M, 
                                    Constants.BACK_LEFT_LOCATION_M, Constants.BACK_RIGHT_LOCATION_M);
        m_swerveDriveOdometry = new SwerveDriveOdometry(m_swerveDriveKinematics, m_ahrs.getRotation2d());

        // Module state setting
        m_frontRightState = new SwerveModuleState(frontRight[0], Rotation2d.fromDegrees(frontRight[1]* 180/Math.PI)); //TODO add rpm to mps method
        m_frontLeftState = new SwerveModuleState(frontLeft[0], Rotation2d.fromDegrees(frontLeft[1]* 180/Math.PI));
        m_backLeftState = new SwerveModuleState(backLeft[0], Rotation2d.fromDegrees(backLeft[1]* 180/Math.PI));
        m_backRightState = new SwerveModuleState(backRight[0], Rotation2d.fromDegrees(backRight[1]* 180/Math.PI));

        SwerveModuleState[] moduleStates = new SwerveModuleState[4];

        moduleStates[0] = m_frontRightState;
        moduleStates[1] = m_frontLeftState;
        moduleStates[2] = m_backLeftState;
        moduleStates[3] = m_backRightState; //TODO figure out module states

        // Reset encoders and gyro to ensure autonomous path following is correct
        this.resetEncoders();
        this.zeroHeading();
    }

    /**
   * Controls movement of robot drivetrain with passed in power and turn values
   * from autonomous input. Example: vision control.
   * Difference from teleopDrive is there's no deadband.
   */
    public void autoDrive(double x1, double y1, double turn)
    {
        this.convertSwerveValues(x1, y1, turn);
    }
    
    public void calculatePIDs() 
    { 
        // m_speedMotorFrontRight.set(m_xController.calculate(Encoder.getDistance(), setpoint)); 
    } 

    // Takes input from analog sticks and convert it into turn and x,y velocities
    // for the wheels
    public void convertSwerveValues(double x1, double y1, double x2) 
    {
        // Width and length of robot
        double w = 25;
        double l = 17;

        // Width and length relative ratios
        double wR;
        double lR;

        // Input velocities and turn
        double vX = 0;
        double vY = 0;
        double turn = 0;

        //implificaton for adding turn and strafe velocity for each wheel
        double a;
        double b;
        double c;
        double d;

        // Apply deadzone to turn analog stick
        if (Math.abs(x2) > 0.2)
            turn = x2;
        
        // Apply dead zone for velocities
        if (Math.abs(x1) > 0.2)
            vX = x1;
        if (Math.abs(y1) > 0.2)
            vY = -y1;

        // Find similar triangles to chassis for turn vectors (radius = 1)
        double turnAngle = Math.atan2(l, w);
        wR = Math.cos(turnAngle);
        lR = Math.sin(turnAngle);

        // Establishing swerve gyro difference
        double gyroCurrent = m_ahrs.getYaw();

        // Adjust strafe vector so that forward constant
        double r = Math.sqrt(vX * vX + vY * vY);
        double strafeAngle = Math.atan2(vY, vX);

        strafeAngle += gyroCurrent / 360 * 2 * Math.PI;
        vX = r * Math.cos(strafeAngle);
        vY = r * Math.sin(strafeAngle);

        // Shortening equations for adding strafe and turn for each wheel
        a = vX - turn * lR;
        b = vX + turn * lR;
        c = vY - turn * wR;
        d = vY + turn * wR;

        // Adjust for exceeding max speed
        double highestSpeed = Math.max(Math.max(Math.max(frontRight[0], frontLeft[0]), backLeft[0]), backRight[0]);

        // Finding speed of each wheel based on x and y velocities
        frontRight[0] = Math.sqrt(Math.abs(b * b + c * c));
        frontLeft[0] = Math.sqrt(Math.abs(b * b + d * d));
        backLeft[0] = Math.sqrt(Math.abs(a * a + d * d));
        backRight[0] = Math.sqrt(Math.abs(a * a + c * c));

        if (highestSpeed > 1) {
            frontRight[0] = frontRight[0] / highestSpeed;
            frontLeft[0] = frontLeft[0] / highestSpeed;
            backLeft[0] = backLeft[0] / highestSpeed;
            backRight[0] = backRight[0] / highestSpeed;
        }

        // Update last angle
        frontRight[2] = frontRight[1];
        frontLeft[2] = frontLeft[1];
        backLeft[2] = backLeft[1];
        backRight[2] = backLeft[1];

        // Set new angles
        if (!(vX == 0 && vY == 0 && turn == 0)) {
            // Find angle of each wheel based on velocities
            frontRight[1] = Math.atan2(c, b) - Math.PI / 2;
            frontLeft[1] = Math.atan2(d, b) - Math.PI / 2;
            backLeft[1] = Math.atan2(d, a) - Math.PI / 2;
            backRight[1] = Math.atan2(c, a) - Math.PI / 2;
        }

        // When a wheel moves more than half a circle in one direction, offsets so it
        // goes the shorter route
        if ((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] < frontRight[1]))
            frontRight[3] -= 2 * Math.PI;
        if ((Math.abs(frontRight[2] - frontRight[1]) > Math.PI && frontRight[2] > frontRight[1]))
            frontRight[3] += 2 * Math.PI;
        if ((Math.abs(frontLeft[2] - frontLeft[1]) > Math.PI && frontLeft[2] < frontLeft[1]))
            frontLeft[3] -= 2 * Math.PI;
        if ((Math.abs(frontLeft[2] - frontLeft[1]) > Math.PI && frontLeft[2] > frontLeft[1]))
            frontLeft[3] += 2 * Math.PI;

        if ((Math.abs(backLeft[2] - backLeft[1]) > Math.PI && backLeft[2] < backLeft[1]))
            backLeft[3] -= 2 * Math.PI;
        if ((Math.abs(backLeft[2] - backLeft[1]) > Math.PI && backLeft[2] > backLeft[1]))
            backLeft[3] += 2 * Math.PI;
        if ((Math.abs(backRight[2] - backRight[1]) > Math.PI && backRight[2] < backRight[1]))
            backRight[3] -= 2 * Math.PI;
        if ((Math.abs(backRight[2] - backRight[1]) > Math.PI && backRight[2] > backRight[1]))
            backRight[3] += 2 * Math.PI;

        drive(m_speedMotorFrontRight, m_angleMotorFrontRight, frontRight[0],
                -(frontRight[1] + frontRight[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
        drive(m_speedMotorFrontLeft, m_angleMotorFrontLeft, frontLeft[0],
                -(frontLeft[1] + frontLeft[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
        drive(m_speedMotorBackLeft, m_angleMotorBackLeft, backLeft[0],
                -(backLeft[1] + backLeft[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);
        drive(m_speedMotorBackRight, m_angleMotorBackRight, backRight[0],
                -(backRight[1] + backRight[3]) / (Math.PI * 2) * Constants.SWERVE_TPR);

        SmartDashboard.putNumber("Current Angle Back Right", backRight[1] + backRight[3]);  
        SmartDashboard.putNumber("Current Angle Back Left", backLeft[1] + backLeft[3]);  
        SmartDashboard.putNumber("Current Angle Front Right", frontRight [1] + frontRight[3]);  
        SmartDashboard.putNumber("Current Angle Front Left", frontLeft[1] + frontLeft[3]);  
    
        SmartDashboard.putNumber("Speed Back Right", backRight[0]);
        SmartDashboard.putNumber("Speed Back Left", backLeft[0]);
        SmartDashboard.putNumber("Speed Front Right", frontRight[0]);
        SmartDashboard.putNumber("Speed Front Left", frontLeft[0]);
    
        SmartDashboard.putNumber("Swerve Yaw", m_ahrs.getYaw());
        SmartDashboard.putNumber("Swerve Compass", m_ahrs.getCompassHeading());
    }

    public void drive(WPI_TalonFX speedMotor, WPI_TalonFX angleMotor, double speed, double angle) 
    {
        
        speedMotor.set(speed * 0.8);

        double setPoint = angle * (Constants.SWERVE_DRIVE_MAX_VOLTAGE * 1.5);

        if (setPoint < 0)
            setPoint += Constants.SWERVE_DRIVE_MAX_VOLTAGE;
        if (setPoint > Constants.SWERVE_DRIVE_MAX_VOLTAGE)
            setPoint -= Constants.SWERVE_DRIVE_MAX_VOLTAGE;

        angleMotor.set(TalonFXControlMode.Position, angle *0.8);

        System.out.println("Speed: " + speed);
        System.out.println("Angle: " + angle);

    }

    public SequentialCommandGroup generateSwerveCommand(Pose2d startingPose, List<Translation2d> wayPoints, 
                                                        Pose2d endingPose, double maxVelocity, boolean isReversed)
    {
        // Voltage constraint so never telling robot to move faster than it is capable of achieving.
        var autoVelocityConstraint =
            new SwerveDriveKinematicsConstraint(m_swerveDriveKinematics, Math.abs(maxVelocity));
        
        // Configuration for trajectory that wraps path constraints.
        TrajectoryConfig trajConfig =
            new TrajectoryConfig(maxVelocity, Constants.MAX_ACCELERATION_MPS2)
            // Add kinematics to track robot speed and ensure max speed is obeyed.
            .setKinematics(m_swerveDriveKinematics)
            // Apply voltage constraint created above.
            .addConstraint(autoVelocityConstraint)
            // Reverse the trajectory based on passed in parameter.
            .setReversed(isReversed);

        // Generate trajectory: initialPose, interiorWaypoints, endPose, trajConfig
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startingPose, wayPoints, endingPose, trajConfig);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, 
            RobotContainer.m_chassis::getPose, m_swerveDriveKinematics, 
            new PIDController(Constants.SPEED_CONTROLLER_KP, 0, 0), 
            new PIDController(Constants.SPEED_CONTROLLER_KP, 0, 0),
            new ProfiledPIDController(Constants.SPEED_CONTROLLER_KP, 0, 0, Constants.ANGLE_CONTROLLER_CONSTRAINTS), 
            RobotContainer.m_chassis::setModuleStates, RobotContainer.m_chassis);
            
        return swerveControllerCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.convertSwerveValues(0, 0, 0)));
    }

    public WPI_TalonFX getMotor(WPI_TalonFX motor)
    {
        return motor;
    }

    public double getHeading()
    {
        return Math.IEEEremainder(m_ahrs.getAngle(), 360) * (Constants.K_GYRO_REVERSED ? -1.0 : 1.0);
    }

    
    public Pose2d getPose()
    {
        return m_swerveDriveOdometry.getPoseMeters();
    }

    // Resets encoders
    public void resetEncoders()
    {
        m_angleMotorFrontRight.setSelectedSensorPosition(0);
        m_speedMotorFrontRight.setSelectedSensorPosition(0);
        m_angleMotorFrontLeft.setSelectedSensorPosition(0);
        m_speedMotorFrontLeft.setSelectedSensorPosition(0);
        m_angleMotorBackLeft.setSelectedSensorPosition(0);
        m_speedMotorBackLeft.setSelectedSensorPosition(0);
        m_angleMotorBackRight.setSelectedSensorPosition(0);
        m_speedMotorBackRight.setSelectedSensorPosition(0);
    }

    public void setModuleStates(SwerveModuleState[] modState)
    {
        moduleStates = modState;
    }

    public void zeroHeading()
    {
        m_ahrs.reset();
        m_ahrs.setAngleAdjustment(0.0);
    }
    // Endregion

} 