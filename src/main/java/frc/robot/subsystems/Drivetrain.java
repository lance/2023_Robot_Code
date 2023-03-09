// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.math.system.plant.LinearSystemId.identifyDrivetrainSystem;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.DifferentialDriveFeedforward;
import edu.wpi.first.math.controller.DifferentialDriveWheelVoltages;
import edu.wpi.first.math.controller.LTVDifferentialDriveController;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.OperatorInterface.DeadZones;
import frc.robot.Constants.kAuto;
import frc.robot.Constants.kDrivetrain.*;
import frc.robot.Constants.kVision;
import frc.robot.commands.PPLTVControllerCommand;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

public class Drivetrain extends SubsystemBase {
  // Initalize motor controllers
  private final CANSparkMax leftLead = new CANSparkMax(CanId.leftDriveLead, MotorType.kBrushless);
  private final CANSparkMax leftFollow =
      new CANSparkMax(CanId.leftDriveFollow, MotorType.kBrushless);
  private final CANSparkMax rightLead = new CANSparkMax(CanId.rightDriveLead, MotorType.kBrushless);
  private final CANSparkMax rightFollow =
      new CANSparkMax(CanId.rightDriveFollow, MotorType.kBrushless);
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
  private LTVDifferentialDriveController ltvController =
      new LTVDifferentialDriveController(
          drivetrainModel,
          Dimensions.trackWidthMeters,
          PathFollowing.qelems,
          PathFollowing.relems,
          20.0 / 1000.0);
  private DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(Dimensions.trackWidthMeters);

  // FF speeds tracking/limiting
  private SlewRateLimiter accelLimiter = new SlewRateLimiter(Rate.maxAccel);
  private SlewRateLimiter angularAccelLimter = new SlewRateLimiter(Rate.maxAngularAccel);
  private DifferentialDriveWheelSpeeds lastSpeeds = new DifferentialDriveWheelSpeeds();

  private Encoder leftEncoder =
      new Encoder(
          Encoders.leftAPort, Encoders.leftBPort, Dimensions.kInvertDrive, EncodingType.k1X);
  private Encoder rightEncoder =
      new Encoder(
          Encoders.rightAPort, Encoders.rightBPort, !Dimensions.kInvertDrive, EncodingType.k1X);
  private DifferentialDriveWheelSpeeds encoderSpeeds = new DifferentialDriveWheelSpeeds();
  private SlewRateLimiter leftVelocityLimiter = new SlewRateLimiter(15);
  private SlewRateLimiter rightVelocityLimiter = new SlewRateLimiter(15);
  private LinearFilter leftSpikeLimiter = LinearFilter.movingAverage(20);
  private LinearFilter rightSpikeLimiter = LinearFilter.movingAverage(20);
  private final AHRS gyro = new AHRS(Port.kMXP);

  // Create vision objects
  private PhotonCamera aprilTagCamera = new PhotonCamera("AprilTagCam");
  private AprilTagFieldLayout aprilTagFieldLayout;
  private PhotonPoseEstimator AprilTagPoseEstimator;

  // Pose estimator
  private DifferentialDrivePoseEstimator DDPoseEstimator;

  // Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Drivetrain");
  private ShuffleboardLayout SBSensors;
  private Field2d robotField2d = new Field2d();

  // Logging
  private DataLog log = DataLogManager.getLog();
  private DoubleArrayLogEntry logEncoderPosition =
      new DoubleArrayLogEntry(log, "Drivetrain/encoderPosition");
  private DoubleArrayLogEntry logEncoderVelocity =
      new DoubleArrayLogEntry(log, "Drivetrain/encoderVelocity");
  private DoubleArrayLogEntry logLastWheelSpeeds =
      new DoubleArrayLogEntry(log, "Drivetrain/lastWheelSpeeds");
  private DoubleArrayLogEntry logVoltages = new DoubleArrayLogEntry(log, "Drivetrain/voltages");
  private DoubleArrayLogEntry logPoseEstimate =
      new DoubleArrayLogEntry(log, "Drivetrain/poseEstimate");
  private DoubleArrayLogEntry logPhotonPose = new DoubleArrayLogEntry(log, "Drivetrain/photonPose");
  private DoubleLogEntry logGyro = new DoubleLogEntry(log, "Drivetrain/gyro");

  // Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    motorInit();
    encoderInit();

    // Load apriltags
    try {
      aprilTagFieldLayout =
          AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
    } catch (Exception e) {
      System.out.println("Failed to load field layout");
    }

    // Start pose estimators
    AprilTagPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP,
            aprilTagCamera,
            kVision.aprilTagCameraPositionTransform);

    DDPoseEstimator =
        new DifferentialDrivePoseEstimator(
            driveKinematics,
            new Rotation2d(getAngle()),
            getLeftDistance(),
            getRightDistance(),
            new Pose2d(),
            PoseEstimator.stateStdDevs,
            PoseEstimator.visionStdDevs);

    shuffleBoardInit();
  }

  // -------------------- Drivetrain commands --------------------

  public Command AutoBalanceCommand() {
    return this.startEnd(
            () -> driveVoltages(kAuto.chargeTipVoltage, kAuto.chargeTipVoltage),
            () -> driveVoltages(0, 0))
        .withTimeout(kAuto.tipTimeout)
        .andThen(
            this.startEnd(
                    () -> driveVoltages(kAuto.chargeCreepVoltage, kAuto.chargeCreepVoltage),
                    () -> driveVoltages(0, 0))
                .until(() -> getRoll() < kAuto.chargeStopAngle)
                .withTimeout(kAuto.creepTimeout));
  }

  public Command followPath(PathPlannerTrajectory path) {
    return new PPLTVControllerCommand(
        path, this::getPose, ltvController, this::getSpeeds, this::driveVoltages, this);
  }

  public Command mobilityAuto() {
    return this.startEnd(() -> driveVoltages(-1, -1), () -> driveVoltages(1, 1))
        .withTimeout(kAuto.mobilityTime)
        .andThen(() -> driveVoltages(0, 0));
  }

  // -------------------- Helpers --------------------

  // Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input) {
    if (Math.abs(input) < DeadZones.teleopDriveDeadZone) return 0;
    return Math.copySign(input * input, input);
  }
  // Enable or disable brake mode on the motors
  public void brakeMode(boolean mode) {
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    leftLead.setIdleMode(nMode);
    leftFollow.setIdleMode(nMode);
    rightLead.setIdleMode(nMode);
    rightFollow.setIdleMode(nMode);
  }

  // -------------------- Public interface methods --------------------

  // Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular
  // speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent) {
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    driveChassisSpeeds(
        new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, Rate.maxAngularSpeed * angularPercent));
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
    var nextChassisSpeeds = driveKinematics.toChassisSpeeds(wheelSpeeds);
    wheelSpeeds =
        driveKinematics.toWheelSpeeds(
            new ChassisSpeeds(
                accelLimiter.calculate(nextChassisSpeeds.vxMetersPerSecond),
                0,
                angularAccelLimter.calculate(nextChassisSpeeds.omegaRadiansPerSecond)));
    driveVoltages(
        DDFeedforward.calculate(
            lastSpeeds.leftMetersPerSecond,
            wheelSpeeds.leftMetersPerSecond,
            lastSpeeds.rightMetersPerSecond,
            wheelSpeeds.rightMetersPerSecond,
            20.0 / 1000.0));
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
    leftMotorGroup.setVoltage(voltages.left);
    rightMotorGroup.setVoltage(voltages.right);
    logVoltages.append(new double[] {voltages.left, voltages.right});
  }

  // Encoder and gyro methods
  public double getLeftDistance() {
    return leftEncoder.getDistance();
  }

  public double getRightDistance() {
    return rightEncoder.getDistance();
  }

  public double getLeftVelocity() {
    return leftEncoder.getRate();
  }

  public double getRightVelocity() {
    return rightEncoder.getRate();
  }

  public double getAngle() {
    return Units.degreesToRadians(-gyro.getYaw());
  }

  public double getPitch() {
    return Units.degreesToRadians(-gyro.getPitch());
  }

  public Pose2d getPose() {
    return DDPoseEstimator.getEstimatedPosition();
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return encoderSpeeds;
  }

  public double getRoll() {
    return gyro.getRoll();
  }

  // -------------------- Periodic methods --------------------

  @Override
  public void periodic() {
    encoderSpeeds =
        new DifferentialDriveWheelSpeeds(
            leftSpikeLimiter.calculate(leftVelocityLimiter.calculate(getLeftVelocity())),
            rightSpikeLimiter.calculate(rightVelocityLimiter.calculate(getRightVelocity())));
    // Update pose estimator with odometry
    DDPoseEstimator.updateWithTime(
        Timer.getFPGATimestamp(),
        new Rotation2d(getAngle()),
        getLeftDistance(),
        getRightDistance());
    robotField2d.setRobotPose(DDPoseEstimator.getEstimatedPosition());

    // Get vision measurement and pass it to pose estimator
    AprilTagPoseEstimator.setReferencePose(DDPoseEstimator.getEstimatedPosition());
    Optional<EstimatedRobotPose> result = AprilTagPoseEstimator.update();
    if (result.isPresent()) {
      DDPoseEstimator.addVisionMeasurement(
          result.get().estimatedPose.toPose2d(), result.get().timestampSeconds);
    }

    // Log shit
    logEncoderPosition.append(new double[] {getLeftDistance(), getRightDistance()});
    logEncoderVelocity.append(new double[] {getLeftVelocity(), getRightVelocity()});
    logGyro.append(getAngle());
    var estimatedPose = DDPoseEstimator.getEstimatedPosition();
    logPoseEstimate.append(
        new double[] {
          estimatedPose.getX(), estimatedPose.getY(), estimatedPose.getRotation().getRadians()
        });
    var photonPose = result.isPresent() ? result.get().estimatedPose.toPose2d() : new Pose2d();
    logPhotonPose.append(
        new double[] {photonPose.getX(), photonPose.getY(), photonPose.getRotation().getRadians()});
    logLastWheelSpeeds.append(
        new double[] {lastSpeeds.leftMetersPerSecond, lastSpeeds.rightMetersPerSecond});
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  // -------------------- Hardware init methods --------------------

  private void motorInit() {
    // Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which
    // group selected by Constants.Drive.kInvertDrive; left = False)
    leftMotorGroup.setInverted(!Dimensions.kInvertDrive);
    rightMotorGroup.setInverted(Dimensions.kInvertDrive);
    leftLead.setSmartCurrentLimit(40);
    leftFollow.setSmartCurrentLimit(40);
    rightLead.setSmartCurrentLimit(40);
    rightFollow.setSmartCurrentLimit(40);
    brakeMode(true);
  }

  private void encoderInit() {
    leftEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    rightEncoder.setDistancePerPulse(
        Dimensions.wheelCircumferenceMeters / (Encoders.gearing * Encoders.PPR));
    leftEncoder.setSamplesToAverage(127);
    rightEncoder.setSamplesToAverage(127);
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
    SBTab.addDouble("Left Voltage", () -> leftLead.getAppliedOutput() * 12);
    SBTab.addDouble("Right Voltage", () -> rightLead.getAppliedOutput() * 12);
    SBTab.addDouble("Left Speed", () -> getSpeeds().leftMetersPerSecond);
    SBTab.addDouble("Right Speed", () -> getSpeeds().rightMetersPerSecond);
  }
}
