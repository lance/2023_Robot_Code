package frc.robot.commands;

import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.utilities.ArmTrajectory;

public class ArmTrajectoryCommand extends CommandBase {
  private Arm arm;
  private ArmTrajectory trajectory;
  private double startTime;

  public ArmTrajectoryCommand(ArmTrajectory trajectory, Arm arm) {
    this.trajectory = trajectory;
    this.arm = arm;
    this.startTime = 0;

    addRequirements(arm);
  }

  @Override
  public void initialize() {
    startTime = WPIUtilJNI.now() * 1e-6;
  }

  @Override
  public void execute() {
    arm.setArmSetpoint(trajectory.sample((WPIUtilJNI.now() * 1e-6) - startTime).state);
  }

  @Override
  public boolean isFinished() {
    return trajectory.getTotalTimeSeconds() > (WPIUtilJNI.now() * 1e-6) - startTime;
  }
}
