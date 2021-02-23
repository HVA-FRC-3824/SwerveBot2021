package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class Constants 
{

    //region CAN IDs

    //Chamber motors IDs
    public static final int chamberDriveID                     = 0;
    public static final int chamberExtenderPortA               = 0;
    public static final int chamberExtenderPortB               = 0;

    //Chassis motor IDs
    public static final int frontRightAngleID                  = 13;
    public static final int frontRightSpeedID                  = 12;
    public static final int frontLeftAngleID                   = 14;
    public static final int frontLeftSpeedID                   = 15;
    public static final int backLeftAngleID                    = 1;
    public static final int backLeftSpeedID                    = 0;
    public static final int backRightAngleID                   = 3;
    public static final int backRightSpeedID                   = 2;

    //Launcher motor IDs
    public static final int launcherTopRightMotorID            = 25; //TODO find right values for motor CAN IDs
    public static final int launcherTopLeftMotorID             = 9;
    public static final int launcherBottomMotorID              = 10;
    public static final int launcherPivotID                    = 11;

    //Intake motor IDs
    public static final int intakeMotorID                      = 6;

    //endregion

    //region Ports

    public static final int driverJoystickPort                 = 0;

    //Intake
    public static final int intakeLeftExtenderPortA            = 1;
    public static final int intakeLeftExtenderPortB            = 2;
    public static final int intakeRightExtenderPortA           = 3;
    public static final int intakeRightExtenderPortB           = 4;

    public static final int intakeMotorTPR                     = 0;
    
    //endregion

    //region PID Values
    
    public static final int k_PIDLoopIDX                       = 0;
    public static final int k_slotIDX                          = 0;
    public static final int k_timeoutMS                        = 30;

    //Chamber PIDs
    public static final int chamberP                           = 0;
    public static final int chamberI                           = 0; 
    public static final int chamberD                           = 0;
    public static final int chamberF                           = 0;

    //Intake PIDs
    public static final int intakeWheelP                       = 0;
    public static final int intakeWheelI                       = 0; 
    public static final int intakeWheelD                       = 0;
    public static final int intakeWheelF                       = 0;

    //Launcher PIDs

    public static final int launcherP                          = 0;
    public static final int launcherI                          = 0;
    public static final int launcherD                          = 0;
    public static final int launcherF                          = 0;

    // Speed Controller PIDs

    public static final double speedControllerkP               = 0;

    // Angle Controller PIDs
    
    public static final double angleControllerkP               = 0;

    //endregion

    //region Subsystem Specific Constants

    //Launcher motors

    public static final int    launcherRPMErrorThreshold       = 500;

    public static final int    launcherPivotMinADC             = 1700;
    public static final int    launcherPivotMaxADC             = 3600;
    public static final double launcherPivotJogMagnitude       = 0.25;
    public static final double launcherPivotErrorThreshold     = 0.5;

    public static final int    launcherPivotADCThreshold       = 50;
    public static final double launcherPivotAngleP             = 0.005;
    public static final double launcherPivotAngleMin           = 0.02;
  
    public static final double launcherAimVisionP              = 0.07;
    public static final double launcherAimVisionMin            = 0.03;
    

    //endregion

    //endregion

    //region Misc

    //region Gyro

    public static final boolean k_gyroReversed                 = true;

    //endregion

    //region Constraints
    
    public static final int    autonomousPathSpeed             = 3;
    
    public static final double intakeMotorPower                = 0.5;

    //Speeds and Constraints
    public static final double swerveDriveMaxVoltage           = 4.95;
    public static final double maxAccelerationMPS2             = 3.0; //meters per second squared
    public static final int swerveGearRatio                    = 3 / 20; //wheel spins per angle motor spin
    public static final int motorTPR                           = 2048; //ticks per revolution of motor
    public static final int swerveTPR                          = motorTPR / swerveGearRatio; //motors ticks per revolution of wheel
    public static final Constraints angleControllerConstraints = new Constraints(0.0, 0.0);

    //location of motors relative to the center
    public static final Translation2d frontRightLocationM      = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y
    public static final Translation2d frontLeftLocationM       = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y
    public static final Translation2d backLeftLocationM        = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y
    public static final Translation2d backRightLocationM       = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y

    //endregion

    //endregion
}
