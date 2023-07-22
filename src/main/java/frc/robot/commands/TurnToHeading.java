// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class TurnToHeading extends CommandBase {

  private final Drive drive;

  private final Rotation2d heading;

  private final PIDController turnPID;

  // private final SimpleMotorFeedforward driveFF;

  private TrapezoidProfile profile;

  private final Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TurnToHeading(Rotation2d heading, Drive drive) {
    this.drive = drive;

    if(DriverStation.getAlliance().equals(Alliance.Red)) {
      this.heading = heading.times(-1.0);
    }
    else {
      this.heading = heading;
    }
    
    timer = new Timer();

    turnPID = new PIDController(AutoConstants.staticTurnP, AutoConstants.staticTurnI, AutoConstants.staticTurnD);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();

    profile = new TrapezoidProfile(new Constraints(AutoConstants.turnRateRadiansPerSecond, AutoConstants.turnAccelRadiansPerSecondSquared), new State(heading.getRadians(), 0.0), new State(drive.getHeading().getRadians(), drive.getTurnRate()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double turnPower = turnPID.calculate(drive.getHeading().getRadians(), profile.calculate(timer.get()).position);

    drive.drive(-turnPower, turnPower);

    SmartDashboard.putNumber("turn power", turnPower);

    SmartDashboard.putNumber("position error", drive.getHeading().minus(new Rotation2d(profile.calculate(timer.get()).position)).getDegrees());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }
}
