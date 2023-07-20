// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private final CANSparkMax leftDrive;
  private final CANSparkMax rightDrive;

  private final RelativeEncoder leftEnc;
  private final RelativeEncoder rightEnc;

  private final DifferentialDriveOdometry odometry;

  /** Creates a new ExampleSubsystem. */
  public Drive() {
    leftDrive = new CANSparkMax(CANDevices.leftDriveId, MotorType.kBrushless);
    rightDrive = new CANSparkMax(CANDevices.rightDriveId, MotorType.kBrushless);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    leftDrive.setIdleMode(IdleMode.kBrake);
    rightDrive.setIdleMode(IdleMode.kBrake);

    leftDrive.setSmartCurrentLimit(60);
    rightDrive.setSmartCurrentLimit(60);

    leftEnc = leftDrive.getEncoder();
    rightEnc = rightDrive.getEncoder();

    leftEnc.setPositionConversionFactor(DriveConstants.encRevToMeters);
    rightEnc.setPositionConversionFactor(DriveConstants.encRevToMeters);

    leftEnc.setVelocityConversionFactor(DriveConstants.encRPMToMetersPerSecond);
    rightEnc.setVelocityConversionFactor(DriveConstants.encRPMToMetersPerSecond);

    odometry = new DifferentialDriveOdometry(
      new Rotation2d(), // FIXME
      leftEnc.getPosition(),
      rightEnc.getPosition()
    );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    odometry.update(
      new Rotation2d(), // FIXME
      leftEnc.getPosition(),
      rightEnc.getPosition()
    );

    SmartDashboard.putNumber("left", leftDrive.get());
    SmartDashboard.putNumber("right", rightDrive.get());

    SmartDashboard.putNumber("x pose", odometry.getPoseMeters().getX());
    SmartDashboard.putNumber("y pose", odometry.getPoseMeters().getY());

    SmartDashboard.putBoolean("braking", leftDrive.getIdleMode().equals(IdleMode.kBrake));

    SmartDashboard.putNumber("output current", (leftDrive.getOutputCurrent() + rightDrive.getOutputCurrent()) / 2.0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double leftPower, double rightPower) {
    leftDrive.set(leftPower);
    rightDrive.set(rightPower);
  }

  public void driveVolts(double leftVolts, double rightVolts) {
    leftDrive.setVoltage(leftVolts);
    rightDrive.setVoltage(rightVolts);
  }

  public void setCurrentLimit(int amps) {
    leftDrive.setSmartCurrentLimit(amps);
    rightDrive.setSmartCurrentLimit(amps);
  }

  public void setBraking(boolean brake) {
    if(brake) {
      leftDrive.setIdleMode(IdleMode.kBrake);
      rightDrive.setIdleMode(IdleMode.kBrake);

      leftDrive.setSmartCurrentLimit(120);
      rightDrive.setSmartCurrentLimit(120);
    }
    else {
      leftDrive.setIdleMode(IdleMode.kCoast);
      rightDrive.setIdleMode(IdleMode.kCoast);

      leftDrive.setSmartCurrentLimit(60);
      rightDrive.setSmartCurrentLimit(60);
    }
  }

  public void setPose(Pose2d pose) {
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);

    odometry.resetPosition(new Rotation2d(), 0.0, 0.0, pose); // FIXME
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
  }

  public double getHeading() {
    return 0.0; // FIXME
  }
}