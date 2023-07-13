package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPreset;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rollers;

public class Auto extends SequentialCommandGroup {
    public Auto(Drive drive, Rollers rollers, Arm arm) {
        super(
            new SetPreset(ArmPreset.Hover, arm),
            new Wait(0.25),
            new OutRoller(rollers),
            new Wait(2.0),
            new StopRollers(rollers),
            new SetPreset(ArmPreset.Stow, arm),
            new Wait(10.0)
            // new DriveTime(-0.4, 2.0, drive)
        );
    }
}
