package frc.robot;

import edu.wpi.first.wpilibj.Joystick;

public class OI 
{
    private static Joystick m_driverJoystick;

    public OI()
    {
        m_driverJoystick = new Joystick(Constants.driverJoystickPort);
    }

    public void configureButtonBindings()
    {
        //Chassis buttons
        
        
    }

    public Joystick getDriverJoystick()
    {
        return m_driverJoystick;
    }
}
