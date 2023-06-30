// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;

public class Rollers extends SubsystemBase {

  private final CANSparkMax rollerMtr;

  private boolean isManual;


  /** Creates a new ExampleSubsystem. */
  public Rollers() {
    rollerMtr = new CANSparkMax(CANDevices.rollerId, MotorType.kBrushless);
    rollerMtr.setInverted(true);
    isManual = false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setPower(double power) {
    if(!isManual) {
      rollerMtr.set(power);
    }
  }

  public void manualControl(double power) {
    if(power != 0.0) {
      isManual = true;
      rollerMtr.set(power);
    }
    else if (isManual) {
      rollerMtr.set(0.0);
      isManual = false;
    }
  }
}
