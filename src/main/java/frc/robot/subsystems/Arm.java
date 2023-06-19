// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPreset;

public class Arm extends SubsystemBase {

  private final CANSparkMax armMtr;

  private final SparkMaxPIDController pid;

  private ArmPreset preset;

  /** Creates a new ExampleSubsystem. */
  public Arm() {
    armMtr = new CANSparkMax(0, MotorType.kBrushless);
    pid = armMtr.getPIDController();
    pid.setOutputRange(-ArmConstants.maxPower, ArmConstants.maxPower);
    preset = ArmPreset.Frame;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public ArmPreset getPreset() {
    return preset;
  }

  public void setPreset(ArmPreset preset) {
    pid.setReference(preset.degrees, ControlType.kPosition);
    this.preset = preset;

  }
}
