package frc.robot.Subsystems;

import java.util.List;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.*;
import edu.wpi.first.wpilibj.trajectory.*;

public class Chassis extends SubsystemBase {

    // region variables

    private AHRS m_ahrs;

    // region Auto

    private SwerveDriveOdometry m_swerveDriveOdometry;
    private SwerveDriveKinematics m_swerveDriveKinematics;

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

    // [Speed, Angle, Previous Angle, Offset]
    private double[] frontRight = { 0, 0, 0, 0 };
    private double[] frontLeft = { 0, 0, 0, 0 };
    private double[] backLeft = { 0, 0, 0, 0 };
    private double[] backRight = { 0, 0, 0, 0 };

    // endregion

    // endregion

    // region methods

    public Chassis() {

        // reset encoders and gyro to ensure autonomous path following is correct

        resetEncoders();
        zeroHeading();

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

        m_swerveDriveKinematics = new SwerveDriveKinematics();
        m_swerveDriveOdometry = new SwerveDriveOdometry(m_swerveDriveKinematics, Rotation2d.fromDegrees(getHeading()));

        // Try to instantiate the navX gyro with exception
        try {
            m_ahrs = new AHRS(SPI.Port.kMXP);
        } catch (RuntimeException ex) {
            System.out.println("\nError instantiating navX-MXP:\n" + ex.getMessage() + "\n");
        }

    }

    // Takes input from analog sticks and convert it into turn and x,y velocities
    // for the wheels
    public void convertSwerveValues(double x1, double y1, double x2) {
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
        if (Math.abs(x2) > 0.15)
            turn = x2;

        // Find similar triangles to chassis for turn vectors (radius = 1)
        double turnAngle = Math.atan2(l, w);
        wR = Math.cos(turnAngle);
        lR = Math.sin(turnAngle);

        // Apply dead zone for velocities
        if (Math.abs(x1) > 0.15)
            vX = x1;
        if (Math.abs(y1) > 0.15)
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
                -(frontRight[1] + frontRight[3]) / (Math.PI * 2) * Constants.motorTicksPerRevolution);
        drive(m_speedMotorFrontLeft, m_angleMotorFrontLeft, frontLeft[0],
                -(frontLeft[1] + frontLeft[3]) / (Math.PI * 2) * Constants.motorTicksPerRevolution);
        drive(m_speedMotorBackLeft, m_angleMotorBackLeft, backLeft[0],
                -(backLeft[1] + backLeft[3]) / (Math.PI * 2) * Constants.motorTicksPerRevolution);
        drive(m_speedMotorBackRight, m_angleMotorBackRight, backRight[0],
                -(backRight[1] + backRight[3]) / (Math.PI * 2) * Constants.motorTicksPerRevolution);

    }

    public void drive(WPI_TalonFX speedMotor, WPI_TalonFX angleMotor, double speed, double angle) {
        speedMotor.set(speed);

        double setPoint = angle * (Constants.swerveDriveMaxVoltage * 1.5);

        if (setPoint < 0)
            setPoint += Constants.swerveDriveMaxVoltage;
        if (setPoint > Constants.swerveDriveMaxVoltage)
            setPoint -= Constants.swerveDriveMaxVoltage;

        angleMotor.set(TalonFXControlMode.Position, angle);

        System.out.println("Speed: " + speed);
        System.out.println("Angle: " + angle);
    }

    public void generateRamsete(Pose2d startingPose, List<Translation2d> waypoints, Pose2d endingPose, double vel,
            boolean isReversed) {
        //TODO Voltage constraint so never telling robot to move faster than it is capable of achieving.

        
        // Configuration for trajectory that wraps path constraints.
        TrajectoryConfig trajConfig =
          new TrajectoryConfig(vel,
                               Constants.k_maxAccelerationMPS2)
              // Add kinematics to track robot speed and ensure max speed is obeyed.
              .setKinematics(Constants.k_driveKinematics)
              //TODO Apply voltage constraint created above

              // Reverse the trajectory based on parameter.
              .setReversed(isReversed);
    
        // Generate trajectory: initialPose, interiorWaypoints, endPose, trajConfig
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
          // Starting pose
          startingPose,
          // Pass through these interior waypoints
          waypoints,
          // Ending pose
          endingPose,
          // Pass config
          trajConfig
        );
    
        /* Create command that will follow the trajectory. */
        /*TODO RamseteCommand ramseteCommand = new RamseteCommand(
          trajectory,
          RobotContainer.m_chassis::getPose,
          new RamseteController(Constants.k_ramseteB, Constants.k_ramseteZeta),
          new SimpleMotorFeedforward(Constants.k_sVolts,
                                     Constants.k_vVoltSPM,
                                     Constants.k_aVoltSPM2),
          Constants.K_DRIVE_KINEMATICS,
          RobotContainer.m_chassis::getWheelSpeeds,
          new PIDController(Constants.k_p_drive_Vel, 0, 0),
          new PIDController(Constants.k_p_drive_Vel, 0, 0),
          RobotContainer.m_chassis::driveWithVoltage, // RamseteCommand passes volts to the callback.
          RobotContainer.m_chassis
        );
    
        // Return command group that will run path following command, then stop the robot at the end. 
        return ramseteCommand.andThen(new InstantCommand(() -> RobotContainer.m_chassis.driveWithVoltage(0, 0)));*/
      }

    public WPI_TalonFX getDriverJoystick(WPI_TalonFX motor)
    {
        return motor;
    }

    //Reset gyro to zero the heading of the robot
    public void zeroHeading()
    {
        m_ahrs.reset();
        m_ahrs.setAngleAdjustment(0.0);
    }

    public double getHeading()
    {
        return Math.IEEEremainder(m_ahrs.getAngle(), 360) * (Constants.k_gyro_reversed ? -1.0 : 1.0);
    }

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

    //endregion

} 