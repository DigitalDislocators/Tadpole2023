// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ArmPreset;
import frc.robot.Constants.ControllerConstants;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.ArmManual;
import frc.robot.commands.InRoller;
import frc.robot.commands.OutRoller;
import frc.robot.commands.RollersManual;
import frc.robot.commands.SetPreset;
import frc.robot.commands.ShootRoller;
import frc.robot.commands.StopRollers;
import frc.robot.commands.TankDrive;
import frc.robot.commands.ToggleBraking;
import frc.robot.commands.auto.BumpHighLow;
import frc.robot.commands.auto.BumpHighMobility;
import frc.robot.commands.auto.High;
import frc.robot.commands.auto.Low;
import frc.robot.commands.auto.SmoothHighLow;
import frc.robot.commands.auto.SmoothHighMobility;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.Rollers;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drive drive = new Drive();
  private final Arm arm = new Arm();
  private final Rollers rollers = new Rollers();

  private final SendableChooser<Command> autoChooser = new SendableChooser<Command>();

  private final CommandJoystick leftJoystick = new CommandJoystick(ControllerConstants.leftJoystickPort);
  private final CommandJoystick rightJoystick = new CommandJoystick(ControllerConstants.rightJoystickPort);
  
  private final XboxController driverController = new XboxController(ControllerConstants.driverControllerPort);
  private final XboxController operatorController = new XboxController(ControllerConstants.operatorController);

  private final Trigger driverRightTrigger = new Trigger(() -> driverController.getRightTriggerAxis() > 0.25);
  private final Trigger driverLeftTrigger = new Trigger(() -> driverController.getLeftTriggerAxis() > 0.25);

  private final JoystickButton operatorAButton = new JoystickButton(operatorController, 1);
  private final JoystickButton operatorBButton = new JoystickButton(operatorController, 2);
  // private final JoystickButton operatorXButton = new JoystickButton(operatorController, 3);
  private final JoystickButton operatorYButton = new JoystickButton(operatorController, 4);
  private final JoystickButton operatorRightBumper = new JoystickButton(operatorController, 6);
  private final JoystickButton operatorLeftBumper = new JoystickButton(operatorController, 5);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    autoChooser.setDefaultOption("DoNothing", null);
    autoChooser.addOption("Low", new Low(rollers, arm));
    autoChooser.addOption("High", new High(rollers, arm));
    autoChooser.addOption("BumpHighMobility", new BumpHighMobility(drive, rollers, arm));
    autoChooser.addOption("BumpHighLow", new BumpHighLow(drive, rollers, arm));
    autoChooser.addOption("SmoothHighMobility", new SmoothHighMobility(drive, rollers, arm));
    autoChooser.addOption("SmoothHighLow", new SmoothHighLow(drive, rollers, arm));
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    if(!DriverStation.isJoystickConnected(ControllerConstants.rightJoystickPort)) {
      drive.setDefaultCommand(new ArcadeDrive(
        () -> deadband(-driverController.getLeftY()), 
        () -> deadband(driverController.getRightX()), 
        true,
        drive
      ));
    }
    else {
      drive.setDefaultCommand(new TankDrive(
        () -> deadband(-leftJoystick.getY()),
        () -> deadband(-rightJoystick.getY()),
        false,
        drive
      ));
    }

    driverRightTrigger.onTrue(new InRoller(rollers)).onFalse(new StopRollers(rollers));
    driverLeftTrigger.whileTrue(new ToggleBraking(drive));

    arm.setDefaultCommand(new ArmManual(() -> deadband(-operatorController.getRightY()), arm));

    rollers.setDefaultCommand(new RollersManual(() -> deadband(operatorController.getRightTriggerAxis() - operatorController.getLeftTriggerAxis()), rollers));

    operatorAButton.onTrue(new SetPreset(ArmPreset.intake, arm));
    operatorBButton.onTrue(new SetPreset(ArmPreset.vertical, arm));
    operatorYButton.onTrue(new SetPreset(ArmPreset.stow, arm));

    operatorRightBumper.onTrue(new ShootRoller(rollers)).onFalse(new StopRollers(rollers));
    operatorLeftBumper.onTrue(new OutRoller(rollers)).onFalse(new StopRollers(rollers));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    // return new DriveStraightDistance(-2.0, 1.0, new Rotation2d(), drive).andThen(new TurnToHeading(new Rotation2d(Math.PI), drive)).andThen(new DriveStraightDistance(-2.0, 1.5, new Rotation2d(Math.PI), drive));
    return autoChooser.getSelected();
  }

  public double deadband(double input) {
    if(Math.abs(input) <= ControllerConstants.controllerDeadband) {
      return 0.0;
    }
    return input;
  }
}
