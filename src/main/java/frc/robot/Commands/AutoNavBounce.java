package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

public class AutoNavBounce extends SequentialCommandGroup
{
    //Creates path from a start point to an end point
    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    Pose2d endingPose = new Pose2d(6.853, 0.056, new Rotation2d(0));

    List<Translation2d> waypoints = List.of(
                                                new Translation2d(    1.07	,	1.336 ),
                                                new Translation2d(    1.583	,	-0.192),
                                                new Translation2d(    2.566	,	-1.521),
                                                new Translation2d(    3.358	,	-0.965),
                                                new Translation2d(    3.321	,	0.149 ),
                                                new Translation2d(    3.401	,	1.262 ),
                                                new Translation2d(    3.599	,	-0.13 ),
                                                new Translation2d(    3.896	,	-1.249),
                                                new Translation2d(    5.146	,	-1.317),
                                                new Translation2d(    5.616	,	-0.198),
                                                new Translation2d(    5.684	,	1.28  ),
                                                new Translation2d(    6.08	,	0.513 )
                                            );

    public AutoNavBounce()
    {
        // Follow path
        RobotContainer.m_chassis.generateSwerveCommand(startingPose, waypoints, endingPose, 2.5, false);
    }
}
