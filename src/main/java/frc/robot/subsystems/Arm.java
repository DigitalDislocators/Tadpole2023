// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmPreset;
import frc.robot.Constants.CANDevices;

public class Arm extends SubsystemBase {

  private final CANSparkMax armMtr;

  private final RelativeEncoder armEnc;

  private final SparkMaxPIDController pid;

  private double target;

  private double preset;

  private boolean isManual;

  /** Creates a new ExampleSubsystem. */
  public Arm() {
    armMtr = new CANSparkMax(CANDevices.armId, MotorType.kBrushless);
    armMtr.setInverted(true);
    armMtr.setSmartCurrentLimit(120);
    armMtr.setIdleMode(IdleMode.kCoast);

    armEnc = armMtr.getEncoder();
    armEnc.setPositionConversionFactor(ArmConstants.degreesToEncRev);
    armEnc.setPosition(-ArmConstants.offsetDeg);

    pid = armMtr.getPIDController();
    pid.setOutputRange(-ArmConstants.maxPower, ArmConstants.maxPower);
    pid.setP(ArmConstants.kP);

    preset = ArmPreset.stow;
    target = ArmPreset.stow;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("arm degrees", armEnc.getPosition());

    if(!isManual) {
      pid.setReference(target, ControlType.kPosition);
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public double getPreset() {
    return preset;
  }

  public void setPreset(double preset) {
    target = preset;
    this.preset = preset;

  }

  public void setPower(double power) {
      if(power != 0.0) {
        isManual = true;
        armMtr.set(power * ArmConstants.maxPower);
      }
      else {
        if(isManual) {
          target = armEnc.getPosition();
        }
        isManual = false;
      }
  }
}
