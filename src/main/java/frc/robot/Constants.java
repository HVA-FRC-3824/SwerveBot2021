package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Constants 
{

    //region CAN IDs

    //Chassis motor IDs
    public static final int frontRightAngleID                                   = 13;
    public static final int frontRightSpeedID                                   = 12;
    public static final int frontLeftAngleID                                    = 14;
    public static final int frontLeftSpeedID                                    = 15;
    public static final int backLeftAngleID                                     = 1;
    public static final int backLeftSpeedID                                     = 0;
    public static final int backRightAngleID                                    = 3;
    public static final int backRightSpeedID                                    = 2;

    //Launcher motor IDs
    public static final int launcherTopRightMotorID                             = 25;
    public static final int launcherTopLeftMotorID                              = 9;
    public static final int launcherBottomWheelID                               = 10;
    public static final int launcherPivotID                                     = 11;

    //Intake motor IDs
    public static final int intakeMotorID                                       = 6;

    //endregion

    //region Ports

    public static final int driverJoystickPort                                  = 0;

    //Intake
    public static final int intakeExtenderPortA                                 = 1;
    public static final int intakeExtenderPortB                                 = 2;
    public static final int intakeMotorTPR                                      = 0;
    
    //endregion

    //region PID Values
    
    public static final int k_PIDLoopIDX                                      = 0;
    public static final int k_slotIDX                                         = 0;
    public static final int k_timeoutMS                                       = 30;

    //Intake PIDs
    public static final int intakeWheelP                                             = 0;
    public static final int intakeWheelI                                             = 0; 
    public static final int intakeWheelD                                             = 0;
    public static final int intakeWheelF                                             = 0;

    //endregion

    //region Motor constants

    //Swerve motors
    public static final double swerveDriveMaxVoltage                               = 4.95;
    public static final int    motorTPR                                            = 2048; //ticks per revolution

    //Launcher motors
    public static final int    launcherRPMErrorThreshold                   = 500;

    public static final int    launcherPivotMinADC                         = 1700;
    public static final int    launcherPivotMaxADC                         = 3600;
    public static final double launcherPivotJogMagnitude                   = 0.25;
    public static final double launcherPivotErrorThreshold                 = 0.5;

    public static final int    launcherPivotADCThreshold                   = 50;
    public static final double launcherPivotAngleP                         = 0.005;
    public static final double launcherPivotAngleMin                       = 0.02;
  
    public static final double launcherAimVisionP                          = 0.07;
    public static final double launcherAimVisionMin                        = 0.03;

    //endregion

    //endregion

    //region Misc

    //region Gyro

    public static final boolean k_gyroReversed                                     = true;

    //endregion

    //region Speeds
    
    public static final float   autonomousPathSpeed                              = 3;

    //endregion

    //endregion
}
