package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI 
{
    private static Joystick m_driverJoystick;

    public static Joystick m_operatorJoystick;

    // Chamber
    private static JoystickButton m_setChamberForwardsBtn;
    private static JoystickButton m_setChamberBackwardsBtn;
    
    // Intake
    private static JoystickButton m_setIntakeWheelRPMBtn;

    // Launcher
    private static JoystickButton m_setLauncherPresetGreenBtn;
    private static JoystickButton m_setLauncherPresetYellowBtn;
    private static JoystickButton m_setLauncherPresetBlueBtn;
    private static JoystickButton m_setLauncherPresetRedBtn;
    
    public OI()
    {
        m_driverJoystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

        m_operatorJoystick = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

        // Chamber buttons
        m_setChamberForwardsBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_FORWARDS_BTN_ID);
        m_setChamberBackwardsBtn  = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BACKWARDS_BTN_ID);

        // Intake buttons
        m_setIntakeWheelRPMBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_INTAKE_WHEEL_RPM_BTN_ID);

        // Launcher buttons
        m_setLauncherPresetGreenBtn    = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_PRESET_GREEN_BTN_ID);
        m_setLauncherPresetYellowBtn   = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_PRESET_YELLOW_BTN_ID);
        m_setLauncherPresetBlueBtn     = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_PRESET_BLUE_BTN_ID);
        m_setLauncherPresetRedBtn      = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_LAUNCHER_PRESET_RED_BTN_ID);

    }

    public void configureButtonBindings()
    {
        // Chamber
        m_setChamberForwardsBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberForwards);
        m_setChamberForwardsBtn.whenReleased(RobotContainer.m_inlineCommands.m_setChamberAuto);
    
        m_setChamberBackwardsBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBackwards);
        m_setChamberBackwardsBtn.whenReleased(RobotContainer.m_inlineCommands.m_setChamberAuto);
        
        // Intake
        m_setIntakeWheelRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setIntakeWheelsRPM);
        m_setIntakeWheelRPMBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopIntakeWheels);

        // Launcher
        m_setLauncherPresetGreenBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
        m_setLauncherPresetGreenBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);
        
        m_setLauncherPresetYellowBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
        m_setLauncherPresetYellowBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);   

        m_setLauncherPresetBlueBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
        m_setLauncherPresetBlueBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

        m_setLauncherPresetRedBtn.whenPressed(RobotContainer.m_inlineCommands.m_setLauncherPreset);
        m_setLauncherPresetRedBtn.whenReleased(RobotContainer.m_inlineCommands.m_stopLaunchSequence);

    }

    public Joystick getDriverJoystick()
    {
        return m_driverJoystick;
    }

    public Joystick getOperatorController()
    {
        return m_operatorJoystick;
    }

}
