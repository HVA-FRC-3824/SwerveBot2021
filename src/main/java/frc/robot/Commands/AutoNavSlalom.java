package frc.robot.Commands;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoNavSlalom extends SequentialCommandGroup
{

    Pose2d startingPose = new Pose2d(0, 0, new Rotation2d(0));
    //List<Translation2d> waypoints = List<>();

    public AutoNavSlalom()
    {
        //follow path
        
    }
}
