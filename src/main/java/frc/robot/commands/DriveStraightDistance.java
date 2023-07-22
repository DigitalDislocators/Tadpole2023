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
public class DriveStraightDistance extends CommandBase {

  private final Drive drive;

  private final double speed;

  private final Rotation2d heading;

  private final PIDController drivePID;

  private final PIDController turnPID;

  // private final SimpleMotorFeedforward driveFF;

  private TrapezoidProfile profile;

  private State goalState;

  private final Timer timer;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public DriveStraightDistance(double distance, double speed, Rotation2d heading, Drive drive) {
    this.drive = drive;
    this.speed = speed;

    if(DriverStation.getAlliance().equals(Alliance.Red)) {
      this.heading = heading.times(-1.0);
    }
    else {
      this.heading = heading;
    }

    goalState = new State(distance, 0.0);

    timer = new Timer();

    drivePID = new PIDController(AutoConstants.driveP, AutoConstants.driveI, AutoConstants.driveD);

    turnPID = new PIDController(AutoConstants.driveTurnP, AutoConstants.driveTurnI, AutoConstants.driveTurnD);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  public DriveStraightDistance(State goalState, double speed, Rotation2d heading, Drive drive) {
    this.drive = drive;
    this.speed = speed;

    if(DriverStation.getAlliance().equals(Alliance.Red)) {
      this.heading = heading.times(-1.0);
    }
    else {
      this.heading = heading;
    }

    this.goalState = goalState;

    timer = new Timer();

    drivePID = new PIDController(AutoConstants.driveP, AutoConstants.driveI, AutoConstants.driveD);

    turnPID = new PIDController(AutoConstants.driveTurnP, AutoConstants.driveTurnI, AutoConstants.driveTurnD);

    turnPID.enableContinuousInput(-Math.PI, Math.PI);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();

    goalState = new State(drive.getAverageDistance() + goalState.position, goalState.velocity);

    profile = new TrapezoidProfile(new Constraints(speed, AutoConstants.accelMetersPerSecondSquared), goalState, new State(drive.getAverageDistance(), drive.getAverageVelocity()));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double drivePower = drivePID.calculate(drive.getAverageDistance(), profile.calculate(timer.get()).position);

    double turnPower = turnPID.calculate(drive.getHeading().getRadians(), heading.getRadians());

    drive.drive(drivePower - turnPower, drivePower + turnPower);

    SmartDashboard.putNumber("position error", drive.getAverageDistance() - profile.calculate(timer.get()).position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
      drive.setBraking(false);
    }
    
    if(goalState.velocity == 0.0)
      drive.drive(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return profile.isFinished(timer.get());
  }
}
