package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.kDrivetrain.Rate;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Indications;
import frc.robot.subsystems.Indications.RobotStates;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * User arcade drive command One joystick for linear (forward/backward) motion and one for angular
 * motion Applies a nonlinear velocity mapping to the joysticks, and limits the acceleration and
 * deceleration of the drivetrain
 */
public class UserArcadeDrive extends CommandBase {
  private final Drivetrain drivetrain;
  private final StringPublisher states;
  private final SlewRateLimiter slewRateLimiter;
  private final DoubleSupplier linearInput;
  private final DoubleSupplier angularInput;
  private final BooleanSupplier boostInput;
  private final BooleanSupplier precisionInput;

  /**
   * Linear supplier and angular supplier are -1 to 1 double inputs that can be passed as method
   * references or lambda in the ctors Same but boolean for boost Drivetrain subsystem instance is
   * passed in
   *
   * @param states
   */
  public UserArcadeDrive(
      DoubleSupplier linearSupplier,
      DoubleSupplier angularSupplier,
      BooleanSupplier boostSupplier,
      BooleanSupplier precisionSupplier,
      Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    slewRateLimiter = new SlewRateLimiter(Rate.driverAccel);
    linearInput = linearSupplier;
    angularInput = angularSupplier;
    boostInput = boostSupplier;
    precisionInput = precisionSupplier;
    states = Indications.getCurrentStateTopic().publish();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Turn off brakemode when command is scheduled
  @Override
  public void initialize() {
    drivetrain.brakeMode(true);
  }

  // Looping body of the command
  @Override
  public void execute() {
    // Clamp the inputs to the correct range, and apply the nonlinear mapping defined in
    // Drivetrain.java
    double xSpeed = Drivetrain.NonLinear(MathUtil.clamp(linearInput.getAsDouble(), -1.0, 1.0));
    double zRotation = Drivetrain.NonLinear(MathUtil.clamp(angularInput.getAsDouble(), -1.0, 1.0));

    // Calculate the linear and rotation speeds requested by the inputs using either the boost(max)
    // range, or the driver range
    var speed = Rate.driverSpeed;
    var rotation = Rate.driverAngularSpeed;
    if (precisionInput.getAsBoolean()) {
      speed = Rate.precisionSpeed;
    } else if (boostInput.getAsBoolean()) {
      speed = Rate.maxSpeed;
      rotation /= 4.0;
    }
    double linearSpeed = xSpeed * speed;
    double angularSpeed = zRotation * rotation;

    // Apply the calculated speeds to the drivetrain
    drivetrain.driveChassisSpeeds(
        new ChassisSpeeds(slewRateLimiter.calculate(linearSpeed), 0, angularSpeed));

    // Indicate drive state with the LEDs
    states.set(RobotStates.DRIVE_FORWARD.name());
  }

  // Unpower the motors when the command ends or is interuppted
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveChassisSpeeds(new ChassisSpeeds(0, 0, 0));
    states.set(RobotStates.OFF.name());
  }
}
