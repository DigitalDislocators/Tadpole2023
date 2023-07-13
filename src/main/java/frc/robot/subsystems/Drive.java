// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.DriveConstants;

public class Drive extends SubsystemBase {

  private final CANSparkMax leftDrive;
  private final CANSparkMax rightDrive;

  /** Creates a new ExampleSubsystem. */
  public Drive() {
    leftDrive = new CANSparkMax(CANDevices.leftDriveId, MotorType.kBrushless);
    rightDrive = new CANSparkMax(CANDevices.rightDriveId, MotorType.kBrushless);

    leftDrive.setInverted(false);
    rightDrive.setInverted(true);

    leftDrive.setIdleMode(IdleMode.kCoast);
    rightDrive.setIdleMode(IdleMode.kCoast);

    leftDrive.setSmartCurrentLimit(60);
    rightDrive.setSmartCurrentLimit(60);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("left", leftDrive.get());
    SmartDashboard.putNumber("right", rightDrive.get());

    SmartDashboard.putBoolean("braking", leftDrive.getIdleMode().equals(IdleMode.kBrake));

    SmartDashboard.putNumber("output current", (leftDrive.getOutputCurrent() + rightDrive.getOutputCurrent()) / 2.0);
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double leftPower, double rightPower) {
    leftDrive.set(leftPower * DriveConstants.maxStraightPower);
    rightDrive.set(rightPower * DriveConstants.maxStraightPower);
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
}