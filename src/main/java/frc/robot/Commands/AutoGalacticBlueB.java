package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoGalacticBlueB extends SequentialCommandGroup
{

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d( 7.94, -0.018, new Rotation2d(0));
    List<Translation2d> waypoints = List.of(
        new Translation2d(    0		    ,        0),
        new Translation2d(    2.226     ,   -0.03 ),
        new Translation2d(    3.965		,    0.031),
        new Translation2d(    5.498		,    1.528),
        new Translation2d(    6.989		,   -0.018)
    );

    public AutoGalacticBlueB()
    {
        addCommands(
        // Extend intake
        new InstantCommand(() -> RobotContainer.m_intake.extendExtender()),
        // Start intake
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.INTAKE_MOTOR_POWER)),

        //TODO Run chamber

        // Follow path to pick up balls
        RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false),

        //TODO Stop chamber

        // Stop intake
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0)),
        // Retract intake
        new InstantCommand(() -> RobotContainer.m_intake.retractExtender())
        );        
    }
}
