package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoGalacticBlueA extends SequentialCommandGroup {

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    List<Translation2d> waypoints = List.of(new Translation2d(0, 0));
    Pose2d endingPose = new Pose2d(0, 0, new Rotation2d(0));

    public AutoGalacticBlueA()
    {
        addCommands(
        new InstantCommand(() -> RobotContainer.m_intake.extendExtender()),
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(Constants.intakeMotorPower)),

        RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false),

        //stop intake
        new InstantCommand(() -> RobotContainer.m_intake.setWheelPower(0)),
        //retract intake
        new InstantCommand(() -> RobotContainer.m_intake.retractExtender())
        );
    }
}
