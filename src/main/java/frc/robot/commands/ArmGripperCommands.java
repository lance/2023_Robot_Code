package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.armState;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Gripper;
import java.util.function.BooleanSupplier;

public class ArmGripperCommands {
  private Arm arm;
  private Gripper gripper;

  public ArmGripperCommands(Arm arm, Gripper gripper) {
    this.arm = arm;
    this.gripper = gripper;
  }

  public Command placeCommad() {
    return arm.simpleMove(0, -0.08)
        .unless(() -> arm.getState() != armState.L3 && arm.getState() != armState.L2)
        .andThen(gripper.ejectCommand());
  }

  public Command intakeCommand(BooleanSupplier gamePieceSelector) {
    return new ConditionalCommand(
            gripper.intakeCommand(GamePiece.CONE),
            gripper.intakeCommand(GamePiece.KUBE),
            gamePieceSelector)
        .andThen(arm.gotoState(armState.HOME));
  }
}
