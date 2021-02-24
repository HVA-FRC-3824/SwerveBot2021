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

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(0, 0, new Rotation2d(0));

    List<Translation2d> waypoints = List.of(
        new Translation2d(     0        ,     0   ),
        new Translation2d(     1.571	,	-0.792),
        new Translation2d(     2.245	,	1.527 ),
        new Translation2d(     4.212	,	1.008 ),
        new Translation2d(     6.365	,	0.061)
        );

    public AutoGalacticBlueA()
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
