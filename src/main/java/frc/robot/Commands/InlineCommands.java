package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

public class InlineCommands
{

    // Declare all inline commands

    // Chamber commands
    public final Command m_runChamber;
    public final Command m_stopChamber;
    
    public final Command m_setChamberForwards;
    public final Command m_setChamberBackwards;
    public final Command m_setChamberAuto;
    
    // Chassis commands
    public final Command m_driveWithJoystick;
    public final Command m_resetHeading;

    // Intake commands
    public final Command m_setIntakeWheelsRPM;
    public final Command m_stopIntakeWheels;

    // Launcher commands
    public final Command m_setLauncherVision; // Turns chassis and sets launcher RPMs
    public final Command m_setLauncherPreset; // Set launcher RPMs to specified setpoint
    public final Command m_stopLaunchSequence; // Enables teleop driving and stops launcher RPMs

    public InlineCommands()
    {
        // Chamber command instantiation
        m_runChamber =
            new InstantCommand(() -> RobotContainer.m_chamber.runChamber(Constants.CHAMBER_MOTOR_POWER));

        m_stopChamber = 
            new InstantCommand(() -> RobotContainer.m_chamber.runChamber(0));

        m_setChamberForwards =
            new RunCommand(() -> RobotContainer.m_chamber.stepChamberDistance(Constants.CHAMBER_BALL_STEP_DIST), RobotContainer.m_chamber);
        m_setChamberBackwards =
            new RunCommand(() -> RobotContainer.m_chamber.stepChamberDistance(-Constants.CHAMBER_BALL_STEP_DIST), RobotContainer.m_chamber);
        m_setChamberAuto =
            new ChamberIndexBalls();

        // Chassis commands instantiation
        m_driveWithJoystick = 
            new RunCommand(() -> 
            RobotContainer.m_chassis.convertSwerveValues    (
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(0),
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(1), 
                    RobotContainer.m_OI.getDriverJoystick().getRawAxis(4)   ), 
                RobotContainer.m_chassis);

        m_resetHeading = 
            new InstantCommand(() -> RobotContainer.m_chassis.zeroHeading());

        // Intake commands instantiation
        m_setIntakeWheelsRPM = 
            new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.5), RobotContainer.m_intake);
        m_stopIntakeWheels = 
            new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0.0), RobotContainer.m_intake);
        
        // Launcher commands instantiation
        m_setLauncherVision =
            new ChassisTurnToTarget().andThen(new InstantCommand(() -> RobotContainer.m_limelight.setModeDriver()));
            //.alongWith(new LauncherAimForTarget(), new InstantCommand(() -> RobotContainer.m_LEDs.setLaunchingStatus(true)))
        m_setLauncherPreset =
            new LauncherSetPreset();
        m_stopLaunchSequence =
            this.m_driveWithJoystick.alongWith(new InstantCommand(() -> RobotContainer.m_launcher.stopLauncher(), RobotContainer.m_launcher));
  
   }
}
