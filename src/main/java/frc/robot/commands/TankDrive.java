// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TankDrive extends CommandBase {

  private final Drive drive;

  private final DoubleSupplier left;
  private final DoubleSupplier right;

  private final boolean isNonLinear;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TankDrive(DoubleSupplier left, DoubleSupplier right, boolean isNonLinear, Drive drive) {
    this.drive = drive;

    this.left = left;
    this.right = right;

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
    if(isNonLinear) {
      double leftPower = left.getAsDouble() * Math.abs(left.getAsDouble());
      double rightPower = right.getAsDouble() * Math.abs(right.getAsDouble());

      drive.drive(leftPower, rightPower);
    }
    else {
      drive.drive(left.getAsDouble(), right.getAsDouble());
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
