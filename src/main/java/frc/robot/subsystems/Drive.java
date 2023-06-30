// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void drive(double leftPower, double rightPower) {
    leftDrive.set(leftPower);
    rightDrive.set(rightPower);
  }
}