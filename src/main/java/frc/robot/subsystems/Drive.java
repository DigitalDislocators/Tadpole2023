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
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private final CANSparkMax leftDrive;
  private final CANSparkMax rightDrive;

  private final RelativeEncoder leftEnc;
  private final RelativeEncoder rightEnc;

  private final ADXRS450_Gyro gyro;

  private Rotation2d gyroOffset;

  // private final DifferentialDriveOdometry odometry;

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

    gyro = new ADXRS450_Gyro();

    gyroOffset = new Rotation2d();

    resetEncoders();

    setHeading(new Rotation2d());

    // odometry = new DifferentialDriveOdometry(
    //   getHeading(),
    //   leftEnc.getPosition(),
    //   rightEnc.getPosition()
    // );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // odometry.update(
    //   getHeading(),
    //   leftEnc.getPosition(),
    //   rightEnc.getPosition()
    // );

    if(DriverStation.isAutonomous()) {
      setCurrentLimit(120);
    }

    SmartDashboard.putNumber("average distance", getAverageDistance());

    SmartDashboard.putNumber("heading", getHeading().getDegrees());

    SmartDashboard.putNumber("turn rate", getTurnRate());

    // SmartDashboard.putNumber("x pose", odometry.getPoseMeters().getX());
    // SmartDashboard.putNumber("y pose", odometry.getPoseMeters().getY());

    SmartDashboard.putNumber("speed (m/s)", (rightEnc.getVelocity() + leftEnc.getVelocity()) / 2.0);

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
    gyro.reset();

    gyroOffset = pose.getRotation();

    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);

    // odometry.resetPosition(getHeading(), 0.0, 0.0, pose);
  }

  public Pose2d getPose() {
    // return odometry.getPoseMeters();
    return null;
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(leftEnc.getVelocity(), rightEnc.getVelocity());
  }

  public double getAverageDistance() {
    return (rightEnc.getPosition());
  }

  public double getAverageVelocity() {
    return (rightEnc.getVelocity() + leftEnc.getVelocity()) * 0.5;
  }

  public Rotation2d getHeading() {
    return gyro.getRotation2d().plus(gyroOffset);
  }

  public void setHeading(Rotation2d heading) {
    gyro.reset();

    gyroOffset = heading;
  }

  public double getTurnRate() {
    return Units.degreesToRadians(gyro.getRate());
  }

  public void resetEncoders() {
    leftEnc.setPosition(0.0);
    rightEnc.setPosition(0.0);
  }
}