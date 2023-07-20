package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rollers;

public class PathTest extends SequentialCommandGroup {
    public PathTest(Drive drive, Rollers rollers, Arm arm) {
        super(
            new SetPose(new Pose2d(1.0, 3.0, new Rotation2d()), drive),
            new FollowTrajectory("New Path", false, drive)
        );
    }
}
