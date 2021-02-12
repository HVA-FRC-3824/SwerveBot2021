package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class Constants 
{

    //region CAN IDs

    //region Chassis motor IDs
    public static final int frontRightAngleID                                   = 13;
    public static final int frontRightSpeedID                                   = 12;
    public static final int frontLeftAngleID                                    = 14;
    public static final int frontLeftSpeedID                                    = 15;
    public static final int backLeftAngleID                                     = 1;
    public static final int backLeftSpeedID                                     = 0;
    public static final int backRightAngleID                                    = 3;
    public static final int backRightSpeedID                                    = 2;
    //endregion

    //region Launcher motor IDs

    public static final int launcherTopRightMotorID                             = 25;
    public static final int launcherTopLeftMotorID                              = 9;
    public static final int launcherBottomWheelID                               = 10;
    public static final int launcherPivotID                                     = 11;

    //endregion

    //endRegion

    //endregion

    //region Ports

    public static final int driverJoystickPort                                  = 0;
    
    //endregion


    //region PID Values

    public static final int k_pIDLoopIDX                                      = 0;
    public static final int k_slotIDX                                          = 0;
    public static final int k_timeoutMS                                       = 30;
    //endregion

    //region Misc

    //region Gyro

    public static final boolean k_gyro_reversed = true;

    //endregion

    //region Speeds
    
    public static final float   autonomousPathSpeed                              =3;

    //endregion

    //region Motors

    //region swerve motors

    public static final double swerveDriveMaxVoltage                            = 4.95;
    public static final int motorTicksPerRevolution                             = 24576;

    //endregion

    //region Launcher motors

    public static final int    launcherTopWheelTPR                         = 2048;
    public static final int    launcherBottomWheelTPR                      = 2048;
    public static final int    launcherRPMErrorThreshold                   = 500;

    public static final int    launcherPivotMinADC                         = 1700;
    public static final int    launcherPivotMaxADC                         = 3600;
    public static final double launcherPivotJogMagnitude                   = 0.25;
    public static final double launcherPivotErrorThreshold                 = 0.5;

    public static final int    launcherPivotADCThreshold                   = 50;
    public static final double launcherPivotAngleP                         = 0.005;
    public static final double launcherPivotAngleMin                       = 0.02;
  
    public static final double LAUNCHER_AIM_VISION_P                          = 0.07;
    public static final double LAUNCHER_AIM_VISION_MIN                        = 0.03;

    //endregion

    //endregion

    //region Voltage things idrk wtf this is ngl

    public static final double k_ramseteB                                     = 2;
    public static final double k_ramseteZeta                                  = 0.7;

    public static final double k_sVolts                                       = 0.372;
    public static final double k_vVoltSPM                                     = 3.09;
    public static final double k_aVoltSPM2                                    = 0.154;
    public static final double k_pDriveVel                                    = 0.00425;
    public static final double k_trackWidthMeters                             = 0.774;
    public static final SwerveDriveKinematics k_driveKinematics               = new SwerveDriveKinematics( new Translation2d(0, 0) );

    public static final double k_maxAccelerationMPS2                          = 3.0;

    //endregion

}
