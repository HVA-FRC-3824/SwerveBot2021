package frc.robot;

import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile.Constraints;

public class Constants 
{

    // Button IDs
    public static final int    OPERATOR_CHAMBER_BASE_RPM_BTN_ID  = 14;

// CAN IDs

    // Chamber motors IDs
    public static final int CHAMBER_WHEEL_ID                     = 0;

    // Chassis motor IDs
    public static final int BACK_LEFT_ANGLE_ID                   = 1;
    public static final int BACK_LEFT_SPEED_ID                   = 0;
    public static final int BACK_RIGHT_ANGLE_ID                  = 3;
    public static final int BACK_RIGHT_SPEED_ID                  = 2;
    public static final int FRONT_RIGHT_ANGLE_ID                 = 13;
    public static final int FRONT_RIGHT_SPEED_ID                 = 12;
    public static final int FRONT_LEFT_ANGLE_ID                  = 14;
    public static final int FRONT_LEFT_SPEED_ID                  = 15;

    // Launcher motor IDs
    public static final int LAUNCHER_BOTTOM_MOTOR_ID             = 10; //TODO find right values for motor CAN IDs
    public static final int LAUNCHER_PIVOT_ID                    = 11;
    public static final int LAUNCHER_TOP_LEFT_MOTOR_ID           = 9;
    public static final int LAUNCHER_TOP_RIGHT_MOTOR_ID          = 25; 

    // Intake motor IDs
    public static final int INTAKE_MOTOR_ID                      = 6;

    // Ports

    // Chamber ports
    public static final int CHAMBER_OUTPUT_PORT_A                = 0;
    public static final int CHAMBER_OUTPUT_PORT_B                = 0;

    // Joystick ports
    public static final int DRIVER_JOYSTICK_PORT                 = 0;
    public static final int OPERATOR_JOYSTICK_PORT               = 1;


    // Intake
    public static final int INTAKE_LEFT_EXTENDER_PORT_A          = 1;
    public static final int INTAKE_LEFT_EXTENDER_PORT_B          = 2;
    public static final int INTAKE_RIGHT_EXTENDER_PORT_A         = 3;
    public static final int INTAKE_RIGHT_EXTENDER_PORT_B         = 4;

    public static final int INTAKE_MOTOR_TPR                     = 0;

    // PID Values
    
    public static final int K_PID_LOOP_IDX                       = 0;
    public static final int K_SLOT_IDX                           = 0;
    public static final int K_TIMEOUT_MS                         = 30;

    // Chamber PIDs
    public static final double CHAMBER_P                            = 0;
    public static final double CHAMBER_I                            = 0; 
    public static final double CHAMBER_D                            = 0;
    public static final double CHAMBER_F                            = 0;

    public static final double K_CHASSIS_ANGLE_P                    = 0;
    public static final double K_CHASSIS_ANGLE_I                    = 0;
    public static final double K_CHASSIS_ANGLE_D                    = 0;

    // Intake PIDs
    public static final double INTAKE_WHEEL_P                       = 0;
    public static final double INTAKE_WHEEL_I                       = 0; 
    public static final double INTAKE_WHEEL_D                       = 0;
    public static final double INTAKE_WHEEL_F                       = 0;

    // Launcher PIDs

    public static final double LAUNCHER_P                           = 0;
    public static final double LAUNCHER_I                           = 0;
    public static final double LAUNCHER_D                           = 0;
    public static final double LAUNCHER_F                           = 0;

    // Speed Controller PIDs

    public static final double SPEED_CONTROLLER_KP               = 0;

    // Angle Controller PIDs
    
    public static final double ANGLE_CONTROLLER_KP               = 0;

    // Launcher motors

    public static final int    LAUNCHER_RPM_ERROR_THRESHOLD      = 500;

    public static final double LAUNCHER_PIVOT_ERROR_THRESHOLD    = 0.5;
    public static final double LAUNCHER_PIVOT_JOG_MAGNITUDE      = 0.25;
    public static final int    LAUNCHER_PIVOT_MAX_ADC            = 3600;
    public static final int    LAUNCHER_PIVOT_MIN_ADC            = 1700;

    public static final int    LAUNCHER_PIVOT_ADC_THRESHOLD      = 50;
    public static final double LAUNCHER_PIVOT_ANGLE_P            = 0.005;
    public static final double LAUNCHER_PIVOT_ANGLE_MIN          = 0.02;
  
    public static final double LAUCHER_AIM_VISION_P              = 0.07;
    public static final double LAUCHER_AIM_VISION_MIN            = 0.03;

    // Gyro

    public static final boolean K_GYRO_REVERSED                  = true;

    // Constraints
    
    public static final int    AUTONOMOUS_PATH_SPEED             = 3;
    
    public static final double CHAMBER_MOTOR_POWER               = 0.5;
    public static final double INTAKE_MOTOR_POWER                = 0.5;

    // Speeds and Constraints
    public static final Constraints ANGLE_CONTROLLER_CONSTRAINTS = new Constraints(0.0, 0.0);   

    public static final double MAX_ACCELERATION_MPS2             = 3.0; //meters per second squared
    public static final int MOTOR_TPR                            = 2048; //ticks per revolution of motor
    public static final double SWERVE_DRIVE_MAX_VOLTAGE          = 4.95;    
    public static final int SWERVE_GEAR_RATIO                    = 3 / 20; //wheel spins per angle motor spin    
    public static final int SWERVE_TPR                           = MOTOR_TPR / SWERVE_GEAR_RATIO; //motors ticks per revolution of wheel


    // Location of motors relative to the center
    public static final Translation2d BACK_LEFT_LOCATION_M       = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y
    public static final Translation2d BACK_RIGHT_LOCATION_M      = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y
    public static final Translation2d FRONT_LEFT_LOCATOIN_M      = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y        
    public static final Translation2d FRONT_RIGHT_LOCATION_M     = new Translation2d(0.7 , 0.3); //TODO forward is +X and left is +Y

    // Miscellaneous Constants
}
