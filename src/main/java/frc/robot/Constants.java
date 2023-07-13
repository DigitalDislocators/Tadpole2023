// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class CANDevices {
    public static final int leftDriveId = 1;
    public static final int rightDriveId = 2;
    public static final int armId = 3;
    public static final int rollerId = 4;

  }

  public static class ControllerConstants {
    public static final int leftJoystickPort = 0;
    public static final int rightJoystickPort = 1;
    public static final double controllerDeadband = 0.1;
    public static final int driverControllerPort = 0;
    public static final int operatorController = 2;
  }

  public static class DriveConstants {
    public static final double maxStraightPower = 1.0;
    public static final double maxTurnPower = 1.0;
  }

  public static class ArmConstants {
    public static final double maxPower = 0.5;
    public static final double kP = 0.02;
    public static final double kD = 0.0;

    public static final double gearReduction = (1.0 / 100.0) * (14.0 / 18.0); // 14 to 18
    public static final double degreesToEncRev = gearReduction * 360.0;
    public static final double offsetDeg = 47.0;

    public static final double minDeg = -36.0;
    public static final double maxDeg = 90.0;
  }

  public enum ArmPreset {
    Stow(-45.0),
    Shoot(-39.0),
    Hover(55.0),
    Wheelie(85.0);

    public final double degrees;

    ArmPreset(double degrees) {
      this.degrees = degrees;
    }
  }

  public static class RollerConstants {
   public static final double inPower = -0.9;
   public static final double outPower = 0.5;
   public static final double shootPower = 1.0;
  }
}
