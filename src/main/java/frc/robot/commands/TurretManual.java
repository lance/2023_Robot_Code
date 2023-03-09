package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kArm.Constraints;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

/**
 * User arcade drive command One joystick for linear (forward/backward) motion and one for angular
 * motion Applies a nonlinear velocity mapping to the joysticks, and limits the acceleration and
 * deceleration of the drivetrain
 */
public class TurretManual extends CommandBase {
  private final Arm arm;
  private final SlewRateLimiter slewRateLimiter;
  private final DoubleSupplier angleInput;

  /**
   * Linear supplier and angular supplier are -1 to 1 double inputs that can be passed as method
   * references or lambda in the ctors Same but boolean for boost Drivetrain subsystem instance is
   * passed in
   */
  public TurretManual(DoubleSupplier angleInput, Arm arm) {
    this.arm = arm;
    slewRateLimiter = new SlewRateLimiter(Constraints.turretAcceleration);
    this.angleInput = angleInput;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    slewRateLimiter.reset(arm.getTurretSetpoint());
  }

  // Looping body of the command
  @Override
  public void execute() {
    arm.setTurretSetpoint(slewRateLimiter.calculate(angleInput.getAsDouble()));
  }

  @Override
  public void end(boolean interrupted) {}
}
