package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class PPLTVControllerCommand extends CommandBase {
  private final Timer timer = new Timer();
  private final PathPlannerTrajectory trajectory;
  private final Supplier<Pose2d> poseSupplier;
  private final LTVDifferentialDriveController controller;
  private final Supplier<DifferentialDriveWheelSpeeds> speedsSupplier;
  private final Consumer<DifferentialDriveWheelVoltages> output;
  private final boolean useAllianceColor;

  private PathPlannerTrajectory transformedTrajectory;

  private static final Field2d logField = new Field2d();
  private static final FieldObject2d desired = logField.getObject("Desired");

  private static Consumer<PathPlannerTrajectory> logActiveTrajectory = null;
  private static Consumer<Pose2d> logTargetPose = null;
  private static Consumer<DifferentialDriveWheelVoltages> logSetpoint = null;
  // private static BiConsumer<Translation2d, Rotation2d> logError =
  //    PPLTVControllerCommand::defaultLogError;

  /**
   * Constructs a new PPRamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param speedsSupplier A function that supplies the speeds of the left and right sides of the
   *     robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param useAllianceColor Should the path states be automatically transformed based on alliance
   *     color? In order for this to work properly, you MUST create your path on the blue side of
   *     the field.
   * @param requirements The subsystems to require.
   */
  public PPLTVControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      LTVDifferentialDriveController controller,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      Consumer<DifferentialDriveWheelVoltages> outputVolts,
      boolean useAllianceColor,
      Subsystem... requirements) {
    this.trajectory = trajectory;
    this.poseSupplier = poseSupplier;
    this.controller = controller;
    this.speedsSupplier = speedsSupplier;
    this.output = outputVolts;
    this.useAllianceColor = useAllianceColor;

    addRequirements(requirements);

    if (useAllianceColor && trajectory.fromGUI && trajectory.getInitialPose().getX() > 8.27) {
      DriverStation.reportWarning(
          "You have constructed a path following command that will automatically transform path states depending"
              + " on the alliance color, however, it appears this path was created on the red side of the field"
              + " instead of the blue side. This is likely an error.",
          false);
    }
  }

  /**
   * Constructs a new PPRamseteCommand that, when executed, will follow the provided trajectory. PID
   * control and feedforward are handled internally, and outputs are scaled -12 to 12 representing
   * units of volts.
   *
   * <p>Note: The controller will *not* set the outputVolts to zero upon completion of the path -
   * this is left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param controller The RAMSETE controller used to follow the trajectory.
   * @param feedforward The feedforward to use for the drive.
   * @param speedsSupplier A function that supplies the speeds of the left and right sides of the
   *     robot drive.
   * @param outputVolts A function that consumes the computed left and right outputs (in volts) for
   *     the robot drive.
   * @param requirements The subsystems to require.
   */
  public PPLTVControllerCommand(
      PathPlannerTrajectory trajectory,
      Supplier<Pose2d> poseSupplier,
      LTVDifferentialDriveController controller,
      Supplier<DifferentialDriveWheelSpeeds> speedsSupplier,
      Consumer<DifferentialDriveWheelVoltages> outputVolts,
      Subsystem... requirements) {
    this(trajectory, poseSupplier, controller, speedsSupplier, outputVolts, false, requirements);
  }

  @Override
  public void initialize() {
    if (useAllianceColor && trajectory.fromGUI) {
      transformedTrajectory =
          PathPlannerTrajectory.transformTrajectoryForAlliance(
              trajectory, DriverStation.getAlliance());
    } else {
      transformedTrajectory = trajectory;
    }

    if (logActiveTrajectory != null) {
      logActiveTrajectory.accept(transformedTrajectory);
    }

    this.timer.reset();
    this.timer.start();

    PathPlannerServer.sendActivePath(transformedTrajectory.getStates());
  }

  @Override
  public void execute() {
    double currentTime = this.timer.get();
    Pose2d currentPose = this.poseSupplier.get();

    PathPlannerTrajectory.PathPlannerState desiredState =
        (PathPlannerTrajectory.PathPlannerState) transformedTrajectory.sample(currentTime);

    logField.setRobotPose(currentPose);
    desired.setPose(desiredState.poseMeters);
    PathPlannerServer.sendPathFollowingData(desiredState.poseMeters, currentPose);

    DifferentialDriveWheelVoltages targetDifferentialDriveWheelVoltages =
        this.controller.calculate(
            currentPose,
            this.speedsSupplier.get().leftMetersPerSecond,
            this.speedsSupplier.get().rightMetersPerSecond,
            desiredState);

    this.output.accept(targetDifferentialDriveWheelVoltages);

    if (logTargetPose != null) {
      logTargetPose.accept(desiredState.poseMeters);
    }

    /*if (logError != null) {
      logError.accept(
          currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
          currentPose.getRotation().minus(desiredState.poseMeters.getRotation()));
    }*/
    defaultLogError(
        currentPose.getTranslation().minus(desiredState.poseMeters.getTranslation()),
        currentPose.getRotation().minus(desiredState.poseMeters.getRotation()),
        this.speedsSupplier.get().leftMetersPerSecond,
        this.speedsSupplier.get().rightMetersPerSecond,
        desiredState);

    if (logSetpoint != null) {
      logSetpoint.accept(targetDifferentialDriveWheelVoltages);
    }
  }

  @Override
  public void end(boolean interrupted) {
    this.timer.stop();

    if (interrupted
        || Math.abs(transformedTrajectory.getEndState().velocityMetersPerSecond) < 0.1) {

      this.output.accept(new DifferentialDriveWheelVoltages());
    }
  }

  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(transformedTrajectory.getTotalTimeSeconds());
  }

  private static void defaultLogError(
      Translation2d translationError,
      Rotation2d rotationError,
      double leftMetersPerSecond,
      double rightMetersPerSecond,
      PathPlannerState desiredState) {
    SmartDashboard.putNumber("PPLTVControllerCommand/xErrorMeters", translationError.getX());
    SmartDashboard.putNumber("PPLTVControllerCommand/yErrorMeters", translationError.getY());
    SmartDashboard.putNumber(
        "PPLTVControllerCommand/rotationErrorDegrees", rotationError.getDegrees());
    SmartDashboard.putData("Blah blah", logField);
    SmartDashboard.putNumber("Left Speed: ", leftMetersPerSecond);
    SmartDashboard.putNumber("Right Speed: ", rightMetersPerSecond);
    SmartDashboard.putNumber("Forward desired: ", desiredState.velocityMetersPerSecond);
    SmartDashboard.putNumber("angular desired: ", desiredState.angularVelocityRadPerSec);
    SmartDashboard.putNumber("Desired accel: ", desiredState.accelerationMetersPerSecondSq);
  }

  /**
   * Set custom logging callbacks for this command to use instead of the default configuration of
   * pushing values to SmartDashboard
   *
   * @param logActiveTrajectory Consumer that accepts a PathPlannerTrajectory representing the
   *     active path. This will be called whenever a PPRamseteCommand starts
   * @param logTargetPose Consumer that accepts a Pose2d representing the target pose while path
   *     following
   * @param logSetpoint Consumer that accepts a ChassisSpeeds object representing the setpoint
   *     speeds
   * @param logError BiConsumer that accepts a Translation2d and Rotation2d representing the error
   *     while path following
   */
  public static void setLoggingCallbacks(
      Consumer<PathPlannerTrajectory> logActiveTrajectory,
      Consumer<Pose2d> logTargetPose,
      Consumer<DifferentialDriveWheelVoltages> logSetpoint,
      BiConsumer<Translation2d, Rotation2d> logError) {
    PPLTVControllerCommand.logActiveTrajectory = logActiveTrajectory;
    PPLTVControllerCommand.logTargetPose = logTargetPose;
    PPLTVControllerCommand.logSetpoint = logSetpoint;
    // PPLTVControllerCommand.logError = logError;
  }
}
