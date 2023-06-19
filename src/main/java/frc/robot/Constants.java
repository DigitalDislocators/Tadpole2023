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
    public static final int leftDriveId = 0;
    public static final int rightDriveid = 0;
    public static final int armId = 0;
    public static final int rollerId = 0;

  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class ArmConstants {
    public static final double maxPower = 0.5;
    public static final double kP = 0.01;
    public static final double kD = 0.0;
  }

  public enum RollerMode {
    Cube,
    Cone
  }

  public enum ArmPreset {
    Stow(-45.0),
    Frame(-30.0),
    Hover(55.0),
    Wheelie(90.0);

    public final double degrees;

    ArmPreset(double degrees) {
      this.degrees = degrees;
    }
  }
}