// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

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

    public static final double gearReduction = 1.0 / 4.0;

    public static final double encRevToMeters = Units.inchesToMeters((4 * Math.PI) * gearReduction);
    public static final double encRPMToMetersPerSecond = Units.inchesToMeters((4 * Math.PI) * gearReduction) / 60.0;
  }

  public static class AutoConstants {
    public static final double ksVolts = 0.12862;
    public static final double kvVoltSecondsPerMeter = 1.5337;
    public static final double kaVoltSecondsSquaredPerMeter = 1.6421; //0.433;

    public static final double kPDriveVel = 0.99721 * 4.0;

    public static final double kTrackWidthMeters = 1.0;
    public static final DifferentialDriveKinematics kDriveKinematics =
      new DifferentialDriveKinematics(kTrackWidthMeters);
    
    public static final double maxVelMetersPerSecond = 0.5;
    public static final double maxAccelMetersPerSecondSq = 1.0;

    public static final double kRamseteB = 2.0 * 0.125 * 0.75;
    public static final double kRamseteZeta = 0.7 * 0.125 * 0.125 * 0.125 * 0.0;

    public static final RamseteController kRamseteCotroller = new RamseteController(kRamseteB, kRamseteZeta);

    public static final double accelMetersPerSecondSquared = 1.0;

    public static final double driveOverBumpMetersPerSecond = 0.5;

    public static final double turnRateRadiansPerSecond = Math.PI;
    public static final double turnAccelRadiansPerSecondSquared = Math.PI * 0.5;

    public static final double driveP = 3.5;
    public static final double driveI = 0.0;
    public static final double driveD = 0.04;

    public static final double driveTurnP = 1.0;
    public static final double driveTurnI = 0.0;
    public static final double driveTurnD = 0.175;

    public static final double staticTurnP = 3.5; // 3.2
    public static final double staticTurnI = 0.0;
    public static final double staticTurnD = 0.24;
  }

  public static class ArmConstants {
    public static final double maxPower = 0.75;
    public static final double kP = 0.02;
    public static final double kD = 0.0;

    public static final double gearReduction = (1.0 / 125.0) * (14.0 / 18.0); // 14 to 18
    public static final double degreesToEncRev = gearReduction * 360.0;
    public static final double offsetDeg = 47.0;

    public static final double minDeg = -45.0;
    public static final double maxDeg = 90.0;
  }

  public class ArmPreset {
    public static final double stow = -45.0;
    public static final double vertical = -15.0;
    public static final double intake = 50.0;
  }

  public static class RollerConstants {
   public static final double inPower = -0.7;
   public static final double outPower = 0.5;
   public static final double shootPower = 1.0;

   public static final int outCurrentLimitAmps = 120;
   public static final int inCurrentLimitAmps = 20;
  }
}
