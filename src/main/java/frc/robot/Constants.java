package frc.robot;

public class Constants 
{

    //region CAN IDs

    //region Chassis motor IDs
    public static final int frontRightAngleID        = 0;
    public static final int frontRightSpeedID        = 1;
    public static final int frontLeftAngleID         = 2;
    public static final int frontLeftSpeedID         = 3;
    public static final int backRightAngleID         = 4;
    public static final int backRightSpeedID         = 5;
    public static final int backLeftAngleID          = 6;
    public static final int backLeftSpeedID          = 7;
    //endregion

    //endRegion

    //endregion

    //region Ports

    public static final int driverJoystickPort = 0;
    
    //endregion


    //region PID Values
    public static final int K_PID_LOOP_IDX           = 0;
    public static final int K_SLOT_IDX               = 0;
    public static final int K_TIMEOUT_MS             = 30;
    //endregion

    //region Misc

    //region Motors

    public static final double swerveDriveMaxVoltage = 4.95;
    public static final int motorTicksPerRevolution  = 24576;

    //endregion

    //endregion

}
