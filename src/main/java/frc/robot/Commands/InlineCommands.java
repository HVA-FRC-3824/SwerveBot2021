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

    // Declare all inline commands

    //region Chassis inline commands

    //region variables

    public final Command m_driveWithJoystick;
    public final Command m_setHeading;

    public final Command m_toggleIntakePistons;

    public final Command m_setIntakeWheelsRPM;
    public final Command m_stopIntakeWheels;

    //endregion

    //region methods

    public InlineCommands()
    {

        // Chassis commands instantiation;
        m_driveWithJoystick = 
            new RunCommand(() -> 
            RobotContainer.m_chassis.convertSwerveValues    (
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(0),
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(1), 
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(2)   ), 
                RobotContainer.m_chassis);

        m_setHeading = 
            new InstantCommand(() -> RobotContainer.m_chassis.zeroHeading());
        

        m_toggleIntakePistons =
            new InstantCommand(() -> RobotContainer.m_intake.toggleExtender());

        m_setIntakeWheelsRPM = 
            new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.5), RobotContainer.m_intake);
        m_stopIntakeWheels = 
            new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0), RobotContainer.m_intake);
    }

    //endregion

    //endregion

}
