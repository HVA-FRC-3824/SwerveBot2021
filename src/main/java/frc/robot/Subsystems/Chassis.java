package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.*;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;

public class Chassis extends SubsystemBase {

    // region variables

    public AHRS m_ahrs;

    // region Auto

    // endregion

    // region motors
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
    private HolonomicDriveController m_holonomicController;

    // [Speed, Angle, Previous Angle, Offset]
    private double[] frontRight = { 0, 0, 0, 0 };
    private double[] frontLeft = { 0, 0, 0, 0 };
    private double[] backLeft = { 0, 0, 0, 0 };
    private double[] backRight = { 0, 0, 0, 0 };

    // endregion

    // endregion

    // region methods

    public Chassis() 
    {

        // Instantiating Drivetrain objects

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

        //Instantiating the Swerve Kinematics & Odometry
        m_swerveDriveKinematics = new SwerveDriveKinematics(Constants.frontRightLocationM, Constants.frontLeftLocationM, 
                                    Constants.backLeftLocationM, Constants.backRightLocationM);
        m_swerveDriveOdometry = new SwerveDriveOdometry(m_swerveDriveKinematics, m_ahrs.getRotation2d());

        //Module state setting
        m_frontRightState = new SwerveModuleState(frontRight[0], Rotation2d.fromDegrees(frontRight[1]* 180/Math.PI)); //TODO add rpm to mps method
        m_frontLeftState = new SwerveModuleState(frontLeft[0], Rotation2d.fromDegrees(frontLeft[1]* 180/Math.PI));
        m_backLeftState = new SwerveModuleState(backLeft[0], Rotation2d.fromDegrees(backLeft[1]* 180/Math.PI));
        m_backRightState = new SwerveModuleState(backRight[0], Rotation2d.fromDegrees(backRight[1]* 180/Math.PI));

        // SwerveModuleState[] moduleStates = m_swerveDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

        m_frontRightState = moduleStates[0];
        m_frontLeftState  = moduleStates[1];
        m_backLeftState   = moduleStates[2];
        m_backRightState  = moduleStates[3];

        

        // Try to instantiate the navX gyro with exception
        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
        }

        // Instantiating PID Controllers
        m_holonomicController = new HolonomicDriveController(m_xController, m_yController, m_angleController);

        // Reset encoders and gyro to ensure autonomous path following is correct
        this.resetEncoders();
        this.zeroHeading();
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

        //
        double a;
        double b;
        double c;
        double d;

        // Apply deadzone to turn analog stick
        if (Math.abs(x2) > 0.2)
            turn = x2;

        // Find similar triangles to chassis for turn vectors (radius = 1)
        double turnAngle = Math.atan2(l, w);
        wR = Math.cos(turnAngle);
        lR = Math.sin(turnAngle);

        // Apply dead zone for velocities
        if (Math.abs(x1) > 0.2)
            vX = x1;
        if (Math.abs(y1) > 0.2)
            vY = -y1;

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

        if (highestSpeed > 1) {
            frontRight[0] = frontRight[0] / highestSpeed;
            frontLeft[0] = frontLeft[0] / highestSpeed;
            backLeft[0] = backLeft[0] / highestSpeed;
            backRight[0] = backRight[0] / highestSpeed;
        }

        // update last angle
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
                -(frontRight[1] + frontRight[3]) / (Math.PI * 2) * Constants.swerveTPR);
        drive(m_speedMotorFrontLeft, m_angleMotorFrontLeft, frontLeft[0],
                -(frontLeft[1] + frontLeft[3]) / (Math.PI * 2) * Constants.swerveTPR);
        drive(m_speedMotorBackLeft, m_angleMotorBackLeft, backLeft[0],
                -(backLeft[1] + backLeft[3]) / (Math.PI * 2) * Constants.swerveTPR);
        drive(m_speedMotorBackRight, m_angleMotorBackRight, backRight[0],
                -(backRight[1] + backRight[3]) / (Math.PI * 2) * Constants.swerveTPR);

    }

    public void drive(WPI_TalonFX speedMotor, WPI_TalonFX angleMotor, double speed, double angle) 
    {
        speedMotor.set(speed * 0.8);

        double setPoint = angle * (Constants.swerveDriveMaxVoltage * 1.5);

        if (setPoint < 0)
            setPoint += Constants.swerveDriveMaxVoltage;
        if (setPoint > Constants.swerveDriveMaxVoltage)
            setPoint -= Constants.swerveDriveMaxVoltage;

        angleMotor.set(TalonFXControlMode.Position, angle *0.8);

        System.out.println("Speed: " + speed);
        System.out.println("Angle: " + angle);
    }

    public SequentialCommandGroup generateSwerveCommand(Pose2d startingPose, List<Translation2d> wayPoints, 
                                                        Pose2d endingPose, double maxVelocity, boolean isReversed)
    {
        //Voltage constraint so never telling robot to move faster than it is capable of achieving.
        var autoVelocityConstraint =
            new SwerveDriveKinematicsConstraint(m_swerveDriveKinematics, Math.abs(maxVelocity));
        
        //Configuration for trajectory that wraps path constraints.
        TrajectoryConfig trajConfig =
            new TrajectoryConfig(maxVelocity, Constants.maxAccelerationMPS2)
            //Add kinematics to track robot speed and ensure max speed is obeyed.
            .setKinematics(m_swerveDriveKinematics)
            //Apply voltage constraint created above.
            .addConstraint(autoVelocityConstraint)
            //Reverse the trajectory based on passed in parameter.
            .setReversed(isReversed);

        //Generate trajectory: initialPose, interiorWaypoints, endPose, trajConfig
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(startingPose, wayPoints, endingPose, trajConfig);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, 
            RobotContainer.m_chassis::getPose, m_swerveDriveKinematics, 
            new PIDController(Constants.speedControllerkP, 0, 0), 
            new PIDController(Constants.speedControllerkP, 0, 0),
            new ProfiledPIDController(Constants.angleControllerkP, 0, 0, Constants.angleControllerConstraints), 
            RobotContainer.m_chassis::setModuleStates, RobotContainer.m_chassis);
            
        return swerveControllerCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.convertSwerveValues(0, 0, 0)));
    }

    public WPI_TalonFX getMotor(WPI_TalonFX motor)
    {
        return motor;
    }

    public double getHeading()
    {
        return Math.IEEEremainder(m_ahrs.getAngle(), 360) * (Constants.k_gyroReversed ? -1.0 : 1.0);
    }

    
    public Pose2d getPose()
    {
        return m_swerveDriveOdometry.getPoseMeters();
    }

    //Resets encoders
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


    // public void updateOdometry() 
    // {
    // //get gyro angle. Negate values to match WPILib convention
    // var gyroAngle = Rotation2d.fromDegrees(-getHeading());
    // m_swerveDriveOdometry.update(gyroAngle, m_frontRightState, m_frontLeftState, m_backLeftState, m_backRightState);
    // //Update odometry
    // }

    //Reset gyro to zero the heading of the robot
    public void zeroHeading()
    {
        m_ahrs.reset();
        m_ahrs.setAngleAdjustment(0.0);
    }
    //endregion

} 