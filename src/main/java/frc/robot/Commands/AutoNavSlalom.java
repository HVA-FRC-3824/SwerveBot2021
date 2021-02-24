package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoNavSlalom extends SequentialCommandGroup
{
    //Creates path from a start point to an end point
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(-0.013, 1.738, new Rotation2d(0));

    List<Translation2d> waypoints = List.of(
                                                new Translation2d(0.946	,	0.037),
                                                new Translation2d(2.567	,	1.806),
                                                new Translation2d(4.237	,	2.084),
                                                new Translation2d(5.622	,	1.336),
                                                new Translation2d(6.234	,	0.216),
                                                new Translation2d(7.496	,	0.105),
                                                new Translation2d(7.886	,	0.971),
                                                new Translation2d(7.187	,	1.818),
                                                new Translation2d(6.012	,	1.2  ),
                                                new Translation2d(5.443	,	0.204),
                                                new Translation2d(3.364	,  -0.068),
                                                new Translation2d(1.818	,	0.427),
                                                new Translation2d(1.057	,	1.385)
                                            );

    public AutoNavSlalom()
    {
        // Follow path
        RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false); 
        
    }
}
