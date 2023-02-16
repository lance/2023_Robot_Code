// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.system.plant.LinearSystemId.identifyDrivetrainSystem;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.DifferentialDriveAccelerationLimiter;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.SerialPort.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kDrivetrain.*;
import frc.robot.Constants.kVision;
import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;

public class Drivetrain extends SubsystemBase {
  // Initalize motor controllers
  private final CANSparkMax leftLead = new CANSparkMax(CanId.leftDriveLead, MotorType.kBrushless);
  private final CANSparkMax leftFollow =
      new CANSparkMax(CanId.leftDriveFollow, MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(CanId.rightDriveLead, MotorType.kBrushless);
  private final CANSparkMax rightFollow =
      new CANSparkMax(CanId.rightDriveFollow, MotorType.kBrushless);

  // Create motor controller groups
  private final MotorControllerGroup leftMotorGroup =
      new MotorControllerGroup(leftLead, leftFollow);
  private final MotorControllerGroup rightMotorGroup =
      new MotorControllerGroup(rightLead, rightFollow);

  // Create Drivetrain controllers and kinematics objects
  private LinearSystem<N2, N2, N2> drivetrainModel =
      identifyDrivetrainSystem(
          Feedforward.Linear.kV,
          Feedforward.Linear.kA,
          Feedforward.Angular.kV,
          Feedforward.Angular.kA,
          Dimensions.trackWidthMeters);
  private DifferentialDriveFeedforward DDFeedforward =
      new DifferentialDriveFeedforward(
          Feedforward.Linear.kV,
          Feedforward.Linear.kA,
          Feedforward.Angular.kV,
          Feedforward.Angular.kA,
          Dimensions.trackWidthMeters);
  private DifferentialDriveWheelSpeeds lastSpeeds = new DifferentialDriveWheelSpeeds();

  private DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(Dimensions.trackWidthMeters);

  private DifferentialDriveAccelerationLimiter accelLimiter =
      new DifferentialDriveAccelerationLimiter(
          drivetrainModel, Dimensions.trackWidthMeters, Rate.maxAccel, Rate.maxAngularAccel);

  private DifferentialDrivePoseEstimator DDPoseEstimator;
  private PIDController leftPIDs = new PIDController(PIDs.Left.kS, PIDs.Left.kV, PIDs.Left.kA);
  private PIDController rightPIDs = new PIDController(PIDs.Right.kS, PIDs.Right.kV, PIDs.Right.kA);

  // Create encoder and gyro objects
  private Encoder leftEncoder = new Encoder(Encoders.leftAPort, Encoders.leftBPort);
  private Encoder rightEncoder = new Encoder(Encoders.rightAPort, Encoders.rightBPort);
  private final AHRS gyro = new AHRS(Port.kMXP);

  // Create vision objects
  private PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCam");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private List<Pair<PhotonCamera, Transform3d>> camList =
      new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private RobotPoseEstimator AprilTagPoseEstimator;

  // Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Pose Estimation");
  private ShuffleboardLayout SBSensors;
  private Field2d robotField2d = new Field2d();

  // Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    // Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which
    // group selected by Constants.Drive.kInvertDrive; left = False)
    leftMotorGroup.setInverted(!Dimensions.kInvertDrive);
    rightMotorGroup.setInverted(Dimensions.kInvertDrive);
    leftEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    rightEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    rightEncoder.setReverseDirection(true);

    // TODO clean up this garbage
    try {
      aprilTagFieldLayout =
          new AprilTagFieldLayout(
              new File(Filesystem.getDeployDirectory(), "HallLayout.json").toPath());
    } catch (Exception e) {
      System.out.println("Failed to load AprilTag Layout");
    }
    camList.add(
        new Pair<PhotonCamera, Transform3d>(
            aprilTagCamera, kVision.aprilTagCameraPositionTransform));
    AprilTagPoseEstimator =
        new RobotPoseEstimator(aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);

    DDPoseEstimator =
        new DifferentialDrivePoseEstimator(
            driveKinematics,
            new Rotation2d(getAngle()),
            getLeftDistance(),
            getRightDistance(),
            new Pose2d(3, 3, new Rotation2d(0)),
            // Local measurement standard deviations. Left encoder, right encoder, gyro.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01),
            // Global measurement standard deviations. X, Y, and theta.
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01));

    shuffleBoardInit();
  }

  // Enable or disable brake mode on the motors
  public void brakeMode(boolean mode) {
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    leftLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
    rightLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
  }

  // Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular
  // speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent) {
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed =
        driveKinematics.toChassisSpeeds(
                new DifferentialDriveWheelSpeeds(Rate.maxSpeed, Rate.maxSpeed))
            .omegaRadiansPerSecond;
    driveChassisSpeeds(
        new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  // Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to
  // drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1, 1);

    driveWheelSpeeds(
        new DifferentialDriveWheelSpeeds(
            Rate.maxSpeed * leftPercent, Rate.maxSpeed * rightPercent));
  }

  // Set the appropriate motor voltages for a desired set of wheel speeds (acceleration limited)
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds) {
    // Feedforward calculated with current velocity and next velcocity with timestep of 20ms
    // (default robot loop period)
    driveLimitedVoltages(
        DDFeedforward.calculate(
            lastSpeeds.leftMetersPerSecond,
            wheelSpeeds.leftMetersPerSecond,
            lastSpeeds.rightMetersPerSecond,
            wheelSpeeds.rightMetersPerSecond,
            20 / 1000));
    lastSpeeds = wheelSpeeds;
  }

  // Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  // (acceleration limited)
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    driveWheelSpeeds(driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  // Drive the motors at a given voltage (doubles)
  public void driveVoltages(double leftVoltage, double rightVoltage) {
    leftMotorGroup.setVoltage(leftVoltage);
    rightMotorGroup.setVoltage(rightVoltage);
  }

  // Drive the motors at a given voltage (DifferentialDriveWheelVoltages)
  public void driveVoltages(DifferentialDriveWheelVoltages voltages) {
    voltages =
        accelLimiter.calculate(
            getLeftVelocity(), getRightVelocity(), voltages.left, voltages.right);

    leftMotorGroup.setVoltage(voltages.left);
    rightMotorGroup.setVoltage(voltages.right);
  }

  // Same as driveVoltages but acceleration is limited according to the drivetrain model
  public void driveLimitedVoltages(DifferentialDriveWheelVoltages voltages) {
    voltages =
        accelLimiter.calculate(
            getLeftVelocity(), getRightVelocity(), voltages.left, voltages.right);

    leftMotorGroup.setVoltage(voltages.left);
    rightMotorGroup.setVoltage(voltages.right);
  }

  // PID Control
  public void FeedforwardPIDControl(
      DifferentialDriveWheelSpeeds wheelSpeeds,
      double leftVelocitySetpoint,
      double rightVelocitySetpoint) {
    // Feedforward calculated with current velocity and next velcocity with timestep of 20ms
    // (default robot loop period)
    var feedforwardVoltages =
        DDFeedforward.calculate(
            getLeftVelocity(),
            wheelSpeeds.leftMetersPerSecond,
            getRightVelocity(),
            wheelSpeeds.rightMetersPerSecond,
            20 / 1000);

    // Command wheel voltages
    driveVoltages(
        leftPIDs.calculate(leftEncoder.getRate(), leftVelocitySetpoint) + feedforwardVoltages.left,
        rightPIDs.calculate(rightEncoder.getRate(), rightVelocitySetpoint)
            + feedforwardVoltages.right);
  }
  // Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input) {
    return Math.copySign(input * input, input);
  }

  // Encoder and gyro methods
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftVelocity() {
    return rightEncoder.getRate();
  }

  public double getRightVelocity() {
    return leftEncoder.getRate();
  }

  public double getAngle() {
    return Units.degreesToRadians(-gyro.getYaw());
  }

  public Pose2d getPose() {
    return DDPoseEstimator.getEstimatedPosition();
  }

  public Trajectory generateTrajectory(Pose2d endPose, ArrayList<Translation2d> waypoints) {

    // Starting Position
    var StartPosition = DDPoseEstimator.getEstimatedPosition();
    // Desired Postion
    var EndPosition = endPose;
    // Empty list of waypoints
    var interiorWaypoints = waypoints;
    // Config
    TrajectoryConfig config =
        new TrajectoryConfig(
                Units.feetToMeters(TrajectoryConstants.kMaxSpeedMetersPerSecond),
                Units.feetToMeters(TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared))
            .setKinematics(driveKinematics);
    config.setReversed(TrajectoryConstants.setReversed);
    // Initialize Traj
    var trajectory =
        TrajectoryGenerator.generateTrajectory(
            StartPosition, interiorWaypoints, EndPosition, config);
    return trajectory;
  }

  private void shuffleBoardInit() {
    SBSensors = SBTab.getLayout("Sensors", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    SBSensors.add("NavX2", gyro).withWidget(BuiltInWidgets.kGyro);
    SBSensors.add("Left Encoder", leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBSensors.add("Right Encoder", rightEncoder).withWidget(BuiltInWidgets.kEncoder);
    SBTab.add("Pose Estimate", robotField2d)
        .withWidget(BuiltInWidgets.kField)
        .withSize(7, 4)
        .withPosition(2, 0);
  }

  @Override
  public void periodic() {
    // Update pose estimator with odometry
    DDPoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(getAngle()),
        getLeftDistance(),
        getRightDistance());
    robotField2d.setRobotPose(DDPoseEstimator.getEstimatedPosition());

    // Get vision measurement and pass it to pose estimator
    AprilTagPoseEstimator.setReferencePose(DDPoseEstimator.getEstimatedPosition());
    Optional<Pair<Pose3d, Double>> result = AprilTagPoseEstimator.update();
    if (result.isPresent()) {
      DDPoseEstimator.addVisionMeasurement(
          result.get().getFirst().toPose2d(), Timer.getFPGATimestamp() - result.get().getSecond());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
