package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class AutoGalacticBlueA extends SequentialCommandGroup {

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    List<Translation2d> wayPoints = List.of(new Translation2d(0, 0));
    Pose2d endingPose = new Pose2d(0, 0, new Rotation2d(0));

    public AutoGalacticBlueA()
    {
        addCommands(
        //start intake

        //run chamber

        //follow path to pick up balls
       // RobotContainer.m_chassis.generateramsete(startingPose, wayPoints, endingPose, Constants.autonomousPathSpeed, false);

        //stop chamber

        //stop intake

        //follow path to finish

    }
}
