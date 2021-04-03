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
    // Instantiate subsystems
    public static final Chamber m_chamber = new Chamber();
    public static final Chassis m_chassis = new Chassis();
    public static final Launcher m_launcher = new Launcher();
    public static final Intake m_intake = new Intake();

    public static final Limelight m_limelight = Limelight.getInstance();
    
    // Region misc

    private final SendableChooser<String> m_autoChooser = new SendableChooser<>();

    // Endregion
    
    // Instantiate OI and inline command
    public static final InlineCommands m_inlineCommands = new InlineCommands();
    public static final OI m_OI = new OI();
    
  //Instantiates RobotContainer
  public RobotContainer()
  {
    m_OI.configureButtonBindings();

    this.initializeStartup();
    
    this.initializeAutoChooser();
  }

  //Initializes the Default Commands for chassis driving
  public static void initializeDefaultCommands()
  {
    m_chassis.setDefaultCommand(m_inlineCommands.m_driveWithJoystick);
    m_chamber.setDefaultCommand(new ChamberIndexBalls());
  }

  //Initializes certain functions when robot starts
  private void initializeStartup()
  {
    //m_limelight.turnOffLED();

    m_chamber.startUltrasonics();
  }

  //Method used to configure Falcon motors (TalonFX motors) to be ran
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

  //Method used to configure Talon SRX motors to be ran
  public static void configureTalonSRX(WPI_TalonSRX talonSRX, boolean controlMode, FeedbackDevice feedbackDevice,
  boolean setInverted, boolean setSensorPhase, double kF, double kP, double kI, double kD, int kCruiseVelocity,
  int kAcceleration, boolean resetPos)
  {
    /* Factory default to reset TalonSRX and prevent unexpected behavior. */
    talonSRX.configFactoryDefault();

    /* Configure Sensor Source for Primary PID. */
    talonSRX.configSelectedFeedbackSensor(feedbackDevice, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);

    /* Configure TalonSRX to drive forward when LED is green. */
    talonSRX.setInverted(setInverted);

    /* Configure TalonSRX's sensor to increment its value as it moves forward. */
    talonSRX.setSensorPhase(setSensorPhase);

    // Determine if the internal PID is being used
    if (controlMode)
    {
        /* Set relevant frame periods (Base_PIDF0 and MotionMagic) to periodic rate
        * (10ms).*/
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.K_TIMEOUT_MS);
      talonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.K_TIMEOUT_MS);
    }

    /**
     * Configure the nominal and peak output forward/reverse.
     * 
     * Nominal Output: minimal/weakest motor output allowed during closed-loop. Peak
     * Output: maximal/strongest motor output allowed during closed-loop.
     */
    talonSRX.configNominalOutputForward(0, Constants.K_TIMEOUT_MS);
    talonSRX.configNominalOutputReverse(0, Constants.K_TIMEOUT_MS);
    talonSRX.configPeakOutputForward(1, Constants.K_TIMEOUT_MS);
    talonSRX.configPeakOutputReverse(-1, Constants.K_TIMEOUT_MS);

    /* Set Motion Magic/Velocity gains (FPID) in slot0. */
    talonSRX.selectProfileSlot(Constants.K_SLOT_IDX, Constants.K_PID_LOOP_IDX);
    talonSRX.config_kF(Constants.K_SLOT_IDX, kF, Constants.K_TIMEOUT_MS);
    talonSRX.config_kP(Constants.K_SLOT_IDX, kP, Constants.K_TIMEOUT_MS);
    talonSRX.config_kI(Constants.K_SLOT_IDX, kI, Constants.K_TIMEOUT_MS);
    talonSRX.config_kD(Constants.K_SLOT_IDX, kD, Constants.K_TIMEOUT_MS);

    // Determine if the internal PID is being used
    if (controlMode)
    {
      /* Set acceleration and cruise velocity for Motion Magic. */
      talonSRX.configMotionCruiseVelocity(kCruiseVelocity, Constants.K_TIMEOUT_MS);
      talonSRX.configMotionAcceleration(kAcceleration, Constants.K_TIMEOUT_MS);
    }

    /* Reset/zero the TalonSRX's sensor. */
    if (resetPos)
    {
      talonSRX.setSelectedSensorPosition(0, Constants.K_PID_LOOP_IDX, Constants.K_TIMEOUT_MS);
    }
  }
    
  /**
   *  Convert RPM to units/100ms for TalonSRX/TalonFX to use for ControlMode.Velocity.
   * @param rpm is desired revolutions per minute.
   * @param tpr is the encoder ticks per revolution.
   */
  public static double convertRPMToVelocity(int rpm, int tpr)
  {
    // (RPM * TPR Units/Revolution / 600 100ms/min)
    return rpm * tpr / 600;
  }

  // Set options for autonomous command choser & display them for selection on the SmartDashboard
  private void initializeAutoChooser()
  {
    // Add command options to chooser
    m_autoChooser.setDefaultOption("DEFAULT COMMAND", "default");
    m_autoChooser.addOption("GALACTIC BLUE A", "galacticBlueA");
    m_autoChooser.addOption("GALACTIC BLUE B", "galacticBlueB");
    m_autoChooser.addOption("GALACTICREDA", "galacticRedA");
    m_autoChooser.addOption("GALACTICREDB", "galacticRedB");
    m_autoChooser.addOption("BARREL", "barrel");
    m_autoChooser.addOption("Bounce", "bounce");
    m_autoChooser.addOption("SLALOM", "slalom");

    // Display chooser on smart dashboard
    SmartDashboard.putData("Autonomous Commands", m_autoChooser);
  }
  
 // Return the command to run during autonomous
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
