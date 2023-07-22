package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPreset;
import frc.robot.Constants.AutoConstants;
import frc.robot.commands.DriveStraightDistance;
import frc.robot.commands.SetHeading;
import frc.robot.commands.SetPreset;
import frc.robot.commands.ShootRoller;
import frc.robot.commands.StopRollers;
import frc.robot.commands.TurnToHeading;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rollers;

public class BumpHighMobility extends SequentialCommandGroup {
    public BumpHighMobility(Drive drive, Rollers rollers, Arm arm) {
        super(
            new SetHeading(new Rotation2d(Math.PI), drive),
            new SetPreset(ArmPreset.stow, arm),
            new Wait(1.0),
            new ShootRoller(rollers),
            new Wait(1.0),
            new StopRollers(rollers),
            new DriveStraightDistance(new State(-1.91, -AutoConstants.driveOverBumpMetersPerSecond), 1.25, new Rotation2d(Math.PI), drive),
            new DriveStraightDistance(new State(-0.73, -AutoConstants.driveOverBumpMetersPerSecond), AutoConstants.driveOverBumpMetersPerSecond, new Rotation2d(Math.PI), drive),
            new DriveStraightDistance(new State(-1.36, 0.0), 1.25, new Rotation2d(Math.PI), drive),
            new TurnToHeading(new Rotation2d(0.0), drive),
            new SetPreset(ArmPreset.intake, arm),
            new Wait(3.0)
        );
    }
}
