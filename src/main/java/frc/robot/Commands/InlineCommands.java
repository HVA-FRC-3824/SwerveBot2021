package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Chassis;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class InlineCommands
{

    //Declare all inline commands

    //region Chassis inline commands

    //region variables

    public final Command m_driveWithJoystick;
    public final Command m_setHeading;

    //endregion

    //region methods

    public InlineCommands()
    {

        //Chassis commands instantiation;
        m_driveWithJoystick = 
            new RunCommand(() -> 
            RobotContainer.m_chassis.convertSwerveValues    (
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(0),
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(1), 
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(4)   ), 
                RobotContainer.m_chassis);

        m_setHeading = 
            new InstantCommand(() -> RobotContainer.m_chassis.zeroHeading());
    }

    //endregion

    //endregion

}
