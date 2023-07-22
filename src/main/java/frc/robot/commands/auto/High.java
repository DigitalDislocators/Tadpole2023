package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ArmPreset;
import frc.robot.commands.SetPreset;
import frc.robot.commands.ShootRoller;
import frc.robot.commands.StopRollers;
import frc.robot.commands.Wait;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Rollers;

public class High extends SequentialCommandGroup {
    public High(Rollers rollers, Arm arm) {
        super(
            new SetPreset(ArmPreset.stow, arm),
            new Wait(1.0),
            new ShootRoller(rollers),
            new Wait(1.0),
            new StopRollers(rollers)
        );
    }
}
