package frc.robot.commands;


import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;
import frc.robot.utilities.SplitSlewRateLimiter;
import edu.wpi.first.math.MathUtil;
import frc.robot.Constants;

/**User arcade drive command
 * One joystick for linear (forward/backward) motion and one for angular motion
 * Applies a nonlinear velocity mapping to the joysticks, and limits the acceleration and deceleration of the drivetrain 
 */
public class UserArcadeDrive extends CommandBase {
  private final Drivetrain m_drivetrain;
  private final SplitSlewRateLimiter m_accelerationLimiter;
  private final DoubleSupplier m_linearInput;
  private final DoubleSupplier m_angularInput;
  private final BooleanSupplier m_boostInput;

  /** Linear supplier and angular supplier are -1 to 1 double inputs that can be passed as method references or lambda in the ctors
    Same but boolean for boost
    Drivetrain subsystem instance is passed in
  */
  public UserArcadeDrive(DoubleSupplier linearSupplier, DoubleSupplier angularSupplier, BooleanSupplier boostSupplier, Drivetrain drivetrain) {
    m_drivetrain = drivetrain;
    m_accelerationLimiter = new SplitSlewRateLimiter(Constants.Drive.Rate.driverAccel, Constants.Drive.Rate.driverDeccel);
    m_linearInput = linearSupplier;
    m_angularInput = angularSupplier;
    m_boostInput = boostSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  //Turn off brakemode when command is scheduled
  @Override
  public void initialize(){
      m_drivetrain.brakeMode(false);
  }

  //Looping body of the command
  @Override
  public void execute(){
    //Clamp the inputs to the correct range, and apply the nonlinear mapping defined in Drivetrain.java
    double xSpeed = Drivetrain.NonLinear(MathUtil.clamp(m_linearInput.getAsDouble(), -1.0, 1.0));
    double zRotation = Drivetrain.NonLinear(MathUtil.clamp(m_angularInput.getAsDouble(), -1.0, 1.0));

    //Calculate the linear and rotation speeds requested by the inputs using either the boost(max) range, or the driver range
    double linearSpeed = xSpeed * (m_boostInput.getAsBoolean() ? Constants.Drive.Rate.maxSpeed : Constants.Drive.Rate.driverSpeed);
    double angularSpeed = zRotation * Constants.Drive.Rate.driverAngularSpeed;

    //Apply the calculated speeds to the drivetrain
    m_drivetrain.driveChassisSpeeds(new ChassisSpeeds((
      m_boostInput.getAsBoolean() ? m_accelerationLimiter.overrideCalculate(linearSpeed): m_accelerationLimiter.calculate(linearSpeed)),
       0, angularSpeed));
  }

  //Unpower the motors when the command ends or is interuppted
  @Override
  public void end(boolean interrupted){
    m_drivetrain.driveChassisSpeeds(new ChassisSpeeds(0,0,0));
  }
}