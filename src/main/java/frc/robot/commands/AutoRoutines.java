package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Gripper;
import java.util.HashMap;
import java.util.Map;

public class AutoRoutines {
  private final Drivetrain drivetrain;
  private final Arm arm;
  private final Gripper gripper;

  private final HashMap<String, Command> routineMap = new HashMap<String, Command>();
  private final SendableChooser<Command> selector = new SendableChooser<Command>();

  public AutoRoutines(Drivetrain drivetrain, Arm arm, Gripper gripper) {
    this.drivetrain = drivetrain;
    this.arm = arm;
    this.gripper = gripper;

    loadRoutines();
    populateSendable();
  }

  /**
   * Get the sendable chooser object
   *
   * @return the sendable chooser object
   */
  public SendableChooser<Command> getChooser() {
    return selector;
  }

  // Auto paths go here
  private void loadRoutines() {
    routineMap.put("No Auto", gripper.intakeCommand());
  }

  // Iterate over hashmap and add routines to sendable
  private void populateSendable() {
    selector.setDefaultOption("No Auto", routineMap.get("No Auto"));
    for (Map.Entry<String, Command> entry : routineMap.entrySet()) {
      selector.addOption(entry.getKey(), entry.getValue());
    }
  }
}
