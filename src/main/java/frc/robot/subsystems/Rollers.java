// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerMode;

public class Rollers extends SubsystemBase {

  private final CANSparkMax rollerMtr;

  private RollerMode mode;

  /** Creates a new ExampleSubsystem. */
  public Rollers() {
    rollerMtr = new CANSparkMax(0, MotorType.kBrushless);
    mode = RollerMode.Cube;
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
    rollerMtr.set(power);
  }
  
  public RollerMode getMode() {
    return mode;
  }

  public void setMode(RollerMode mode) {
    this.mode = mode;
  }
}
