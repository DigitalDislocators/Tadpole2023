// Copyturn (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ArcadeDrive extends CommandBase {

  private final Drive drive;

  private final DoubleSupplier straight;
  private final DoubleSupplier turn;

  private final boolean isNonLinear;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public ArcadeDrive(DoubleSupplier straight, DoubleSupplier turn, boolean isNonLinear, Drive drive) {
    this.drive = drive;

    this.straight = straight;
    this.turn = turn;

    this.isNonLinear = isNonLinear;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(DriverStation.isTeleop()) {
      double straightPower;
      double turnPower;

      if(isNonLinear) {
        straightPower = straight.getAsDouble() * Math.abs(straight.getAsDouble());
        turnPower = turn.getAsDouble() * Math.abs(turn.getAsDouble());
      }
      else {
        straightPower = straight.getAsDouble();
        turnPower = turn.getAsDouble();
      }

      turnPower *= DriveConstants.maxTurnPower;

      if(Math.abs(straightPower) < 0.1) {
        drive.setCurrentLimit(120);
      }
      else {
        drive.setCurrentLimit(60);
      }

      drive.drive(straightPower + turnPower, straightPower - turnPower);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
