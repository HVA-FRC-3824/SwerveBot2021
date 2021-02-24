package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class OI 
{
    private static Joystick m_driverJoystick;

    public static Joystick m_operatorJoystick;

    // Chamber
    private static JoystickButton m_setChamberBaseRPMBtn;

    public OI()
    {
        m_driverJoystick = new Joystick(Constants.DRIVER_JOYSTICK_PORT);

        m_operatorJoystick = new Joystick(Constants.OPERATOR_JOYSTICK_PORT);

        // Chamber
        
        m_setChamberBaseRPMBtn = new JoystickButton(m_operatorJoystick, Constants.OPERATOR_CHAMBER_BASE_RPM_BTN_ID);
    }

    public void configureButtonBindings()
    {
        // Chassis buttons
        //TODO implement this in in line commands m_setChamberBaseRPMBtn.whenPressed(RobotContainer.m_inlineCommands.m_setChamberBaseRPM);
        
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
