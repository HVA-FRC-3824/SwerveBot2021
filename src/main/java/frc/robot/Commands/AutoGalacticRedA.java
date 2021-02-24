package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoGalacticRedA extends SequentialCommandGroup
{
  //Creates path from a start point to an end point
  Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(6.365, 0.061, new Rotation2d(0));

    List<Translation2d> waypoints = List.of(
                                              new Translation2d(     1.571	,	-0.792),
                                              new Translation2d(     2.245	,	1.527 ),
                                              new Translation2d(     4.212	,	1.008 )
                                            );

    public AutoGalacticRedA()
    {
      addCommands(
        // Extend intake
        new InstantCommand(() -> RobotContainer.m_intake.extendExtender()),
        // Start intake
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.INTAKE_MOTOR_POWER)),

        //Run chamber
        new InstantCommand(() -> RobotContainer.m_chamber.runChamber(Constants.CHAMBER_MOTOR_POWER)),

        // Follow path to pick up balls
        RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false),

        //Stop chamber
        new InstantCommand(() -> RobotContainer.m_chamber.runChamber(0)),

        // Stop intake
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0)),
        // Retract intake
        new InstantCommand(() -> RobotContainer.m_intake.retractExtender())
        );   
    }
}
