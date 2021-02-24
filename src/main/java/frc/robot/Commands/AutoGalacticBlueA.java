package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class AutoGalacticBlueA extends SequentialCommandGroup {

    //Creates path from a start point to an end point
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(7.799,-0.37, new Rotation2d(0));
    
    List<Translation2d> waypoints = List.of(
                                                new Translation2d(1.15	,	-0.123),
                                                new Translation2d(2.344	,	-0.6),
                                                new Translation2d(3.952	,	-0.717),
                                                new Translation2d(4.373	,	0.458),
                                                new Translation2d(4.756	,	1.596),
                                                new Translation2d(6.16	,	0.792)
                                            );

    public AutoGalacticBlueA()
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
