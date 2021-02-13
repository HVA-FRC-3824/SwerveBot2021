package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import frc.robot.Commands.*;
import frc.robot.Subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer 
{

    //instantiate subsystems
    public static final Chassis m_chassis = new Chassis();

    public static final Launcher m_launcher = new Launcher();

    public static final Intake m_intake = new Intake();

    //region misc

    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

    //endregion
    
    //instantiate 
    public static final OI m_OI = new OI();
    public static final InlineCommands m_inlineCommands = new InlineCommands();

    public static void initializeDefaultCommands()
    {
      m_chassis.setDefaultCommand(m_inlineCommands.m_driveWithJoystick);
    }

    public static void configureTalonFX(WPI_TalonFX talonFX, boolean setInverted, boolean setSensorPhase, double kF, double kP, double kI, double kD) 
    {
        /* Factory default to reset TalonFX and prevent unexpected behavior. */
        talonFX.configFactoryDefault();

        /* Configure Sensor Source for Primary PID. */
        talonFX.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.k_PIDLoopIDX,
            Constants.k_timeoutMS);

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
        talonFX.configNominalOutputForward(0, Constants.k_timeoutMS);
        talonFX.configNominalOutputReverse(0, Constants.k_timeoutMS);
        talonFX.configPeakOutputForward(1, Constants.k_timeoutMS);
        talonFX.configPeakOutputReverse(-1, Constants.k_timeoutMS);

        /* Set the Velocity gains (FPID) in slot0. */
        talonFX.selectProfileSlot(Constants.k_slotIDX, Constants.k_PIDLoopIDX);
        talonFX.config_kF(Constants.k_slotIDX, kF, Constants.k_timeoutMS);
        talonFX.config_kP(Constants.k_slotIDX, kP, Constants.k_timeoutMS);
        talonFX.config_kI(Constants.k_slotIDX, kI, Constants.k_timeoutMS);
        talonFX.config_kD(Constants.k_slotIDX, kD, Constants.k_timeoutMS);

        /**
         * Reset/zero the TalonFX's sensor. Will be required for implementation into
         * chassis (position considered), but not launcher (velocity only).
         */
        talonFX.setSelectedSensorPosition(0, Constants.k_PIDLoopIDX, Constants.k_timeoutMS);
    }

    public static void configureTalonSRX(WPI_TalonSRX talonSRX, boolean controlMode, FeedbackDevice feedbackDevice,
    boolean setInverted, boolean setSensorPhase, double kF, double kP, double kI, double kD, int kCruiseVelocity,
    int kAcceleration, boolean resetPos)  
    {
        //Factory default to reset TalonSRX and prevent unexpected behavior.
        talonSRX.configFactoryDefault();
    
        // Configure Sensor Source for Primary PID.
        talonSRX.configSelectedFeedbackSensor(feedbackDevice, Constants.k_PIDLoopIDX, Constants.k_timeoutMS);
    
        // Configure TalonSRX to drive forward when LED is green.
        talonSRX.setInverted(setInverted);
    
        // Configure TalonSRX's sensor to increment its value as it moves forward.
        talonSRX.setSensorPhase(setSensorPhase);
    
        // Determine if the internal PID is being used
        if (controlMode)
        {
           /* Set relevant frame periods (Base_PIDF0 and MotionMagic) to periodic rate
           * (10ms).*/
          talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.k_timeoutMS);
          talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.k_timeoutMS);
        }
    
        /**
         * Configure the nominal and peak output forward/reverse.
         * 
         * Nominal Output: minimal/weakest motor output allowed during closed-loop. Peak
         * Output: maximal/strongest motor output allowed during closed-loop.
         */
        talonSRX.configNominalOutputForward(0, Constants.k_timeoutMS);
        talonSRX.configNominalOutputReverse(0, Constants.k_timeoutMS);
        talonSRX.configPeakOutputForward(1, Constants.k_timeoutMS);
        talonSRX.configPeakOutputReverse(-1, Constants.k_timeoutMS);
    
        /* Set Motion Magic/Velocity gains (FPID) in slot0. */
        talonSRX.selectProfileSlot(Constants.k_slotIDX, Constants.k_PIDLoopIDX);
        talonSRX.config_kF(Constants.k_slotIDX, kF, Constants.k_timeoutMS);
        talonSRX.config_kP(Constants.k_slotIDX, kP, Constants.k_timeoutMS);
        talonSRX.config_kI(Constants.k_slotIDX, kI, Constants.k_timeoutMS);
        talonSRX.config_kD(Constants.k_slotIDX, kD, Constants.k_timeoutMS);
    
        // Determine if the internal PID is being used
        if (controlMode)
        {
          /* Set acceleration and cruise velocity for Motion Magic. */
          talonSRX.configMotionCruiseVelocity(kCruiseVelocity, Constants.k_timeoutMS);
          talonSRX.configMotionAcceleration(kAcceleration, Constants.k_timeoutMS);
        }
    
        /* Reset/zero the TalonSRX's sensor. */
        if (resetPos)
        {
          talonSRX.setSelectedSensorPosition(0, Constants.k_PIDLoopIDX, Constants.k_timeoutMS);
        }
      }
    
   /* Convert RPM to units/100ms for TalonSRX/TalonFX to use for ControlMode.Velocity.
   * @param rpm is desired revolutions per minute.
   * @param tpr is the encoder ticks per revolution.*/
  public static double convertRPMToVelocity(int rpm, int tpr)
  {
    // (RPM * TPR Units/Revolution / 600 100ms/min)
    return rpm * tpr / 600;
  }

  //Set options for autonomous command choser & display them for selection on the SmartDashboard
  private void initializeAutoChooser()
  {
    //Add command options to chooser
    m_autoChooser.setDefaultOption("DEFAULT COMMAND", "default");
    m_autoChooser.addOption("GALACTIC BLUE A", "galacticBlueA");
    m_autoChooser.addOption("GALACTIC BLUE B", "galacticBlueB");
    m_autoChooser.addOption("GALACTICREDA", "galacticRedA");
    m_autoChooser.addOption("GALACTICREDB", "galacticRedB");
    m_autoChooser.addOption("BARREL", "barrel");
    m_autoChooser.addOption("Bounce", "bounce");
    m_autoChooser.addOption("SLALOM", "slalom");

    //Display chooser on smart dashboard

    SmartDashboard.putData("Autonomous Commands", m_autoChooser);
  }
    //Return the command to run during autonomous
  public Command getAutonomousCommand ()
  {
    switch (m_autoChooser.getSelected())
    {
      case "default":
        return null;
      case "galacticBlueA":
        return new AutoGalacticBlueA();
      case "galacticBlueB":
        return new AutoGalacticBlueB();
      case "galacticRedA":
        return new AutoGalacticRedA();
      case "galacticRedB":
        return new AutoGalacticRedB();
      case "barrel":
        return new AutoNavBarrel();
      case "bounce":
        return new AutoNavBounce();
      case "slalom":
        return new AutoNavSlalom();
      default:
        System.out.println("\nError selecting autonomous command:\nCommand selected: "+m_autoChooser.getSelected()+"\n");
        return null;
    }
  }

}
