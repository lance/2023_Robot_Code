// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorInterface;
import frc.robot.Constants.OperatorInterface.Bindings;
import frc.robot.Constants.armState;
import frc.robot.commands.ArmGripperCommands;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.TurretManual;
import frc.robot.commands.UserArcadeDrive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorInterface.primaryController);
  private final CommandJoystick armJoystick =
      new CommandJoystick(OperatorInterface.secondaryController);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Arm arm = new Arm();
  private final Gripper gripper = new Gripper();
  private final ArmGripperCommands armGripperCommands = new ArmGripperCommands(arm, gripper);
  private final AutoRoutines autoRoutines = new AutoRoutines(drivetrain, arm, gripper);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    arm.encoderStartCommand().schedule();
    drivetrain.setDefaultCommand(
        new UserArcadeDrive(
            () -> -driverController.getLeftY(),
            () -> -driverController.getRightX(),
            () -> driverController.getLeftTriggerAxis() > 0.1,
            () -> driverController.getRightTriggerAxis() > 0.1,
            drivetrain));

    // Post sendable chooser for auto
    Shuffleboard.getTab("Auto").add("Auto selector", autoRoutines.getChooser()).withSize(3, 1);

    // Configure the trigger bindings
    configureBindings();
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
    armJoystick.button(Bindings.home).onTrue(arm.gotoState(armState.HOME));
    armJoystick.button(Bindings.ground).onTrue(arm.gotoState(armState.GROUND));
    armJoystick.button(Bindings.L2).onTrue(arm.gotoState(armState.L2));
    armJoystick.button(Bindings.L3).onTrue(arm.gotoState(armState.L3));
    armJoystick.button(Bindings.doublesub).onTrue(arm.gotoState(armState.DOUBLESUB));

    armJoystick
        .button(Bindings.intake)
        .toggleOnTrue(armGripperCommands.intakeCommand(() -> armJoystick.getThrottle() < 0.5));

    armJoystick.button(Bindings.place).onTrue(armGripperCommands.placeCommad());

    armJoystick.pov(0).whileTrue(new TurretManual(() -> -0.5 * armJoystick.getTwist(), arm));
    armJoystick.pov(180).onTrue(arm.turretHome());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutoCommand() {
    return autoRoutines.getChooser().getSelected();
  }

  public Command getTelopInitCommand() {
    return arm.gotoState(armState.HOME);
  }
}
