package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class RobotContainer 
{
    public static void configureTalonFX(WPI_TalonFX talonFX, boolean setInverted, boolean setSensorPhase, double kF, double kP, double kI, double kD) 
    {
        /* Factory default to reset TalonFX and prevent unexpected behavior. */
        talonFX.configFactoryDefault();

        /* Configure Sensor Source for Primary PID. */
        talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.K_PID_LOOP_IDX,
            Constants.K_TIMEOUT_MS);

        /* Configure TalonFX to drive forward when LED is green. */
        talonFX.setInverted(setInverted);
        /* Configure TalonFX's sensor to increment its value as it moves forward. */
        talonFX.setSensorPhase(setSensorPhase);
        
        /**
         * Configure the nominal and peak output forward/reverse.
         * 
         * Nominal Output: minimal/weakest motor output allowed during closed-loop. Peak
         * Output: maximal/strongest motor output allowed during closed-loop.
         */
        talonFX.configNominalOutputForward(0, Constants.K_TIMEOUT_MS);
        talonFX.configNominalOutputReverse(0, Constants.K_TIMEOUT_MS);
        talonFX.configPeakOutputForward(1, Constants.K_TIMEOUT_MS);
        talonFX.configPeakOutputReverse(-1, Constants.K_TIMEOUT_MS);

        /* Set the Velocity gains (FPID) in slot0. */
        talonFX.selectProfileSlot(Constants.K_SLOT_IDX, Constants.K_PID_LOOP_IDX);
        talonFX.config_kF(Constants.K_SLOT_IDX, kF, Constants.K_TIMEOUT_MS);
        talonFX.config_kP(Constants.K_SLOT_IDX, kP, Constants.K_TIMEOUT_MS);
        talonFX.config_kI(Constants.K_SLOT_IDX, kI, Constants.K_TIMEOUT_MS);
        talonFX.config_kD(Constants.K_SLOT_IDX, kD, Constants.K_TIMEOUT_MS);

        /**
         * Reset/zero the TalonFX's sensor. Will be required for implementation into
         * chassis (position considered), but not launcher (velocity only).
         */
        talonFX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
    }
}
