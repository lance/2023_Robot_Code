// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//General
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;

//Motors
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.IdleMode;

//Kinematics and drivetrain abstractions
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;

//Sensors
import edu.wpi.first.wpilibj.Encoder;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort.Port;

//Vision
import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;
import org.photonvision.RobotPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;
import java.io.File;
import java.util.ArrayList;
import edu.wpi.first.math.Pair;
import java.util.List;
import java.util.Optional;

//Shuffleboard
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

//Math
import java.lang.Math;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.Nat;

//Constants
import frc.robot.Constants.Drivetrain.*;
import frc.robot.Constants.CanId;
import frc.robot.Constants.Vision;

//Traj Gen
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final CANSparkMax m_leftLead = new CANSparkMax(CanId.leftDriveLead, MotorType.kBrushless);
  private final CANSparkMax m_leftFollow = new CANSparkMax(CanId.leftDriveFollow,MotorType.kBrushless);
  private final CANSparkMax m_rightLead = new CANSparkMax(CanId.rightDriveLead,MotorType.kBrushless);
  private final CANSparkMax m_rightFollow = new CANSparkMax(CanId.rightDriveFollow,MotorType.kBrushless);
  //Create motor controller groups
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead, m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead, m_rightFollow);

  //Create Drivetrain controllers and kinematics objects
  private SimpleMotorFeedforward m_leftFeedforward = new SimpleMotorFeedforward(Feedforward.Left.kS, Feedforward.Left.kV, Feedforward.Left.kA);
  private SimpleMotorFeedforward m_rightFeedforward = new SimpleMotorFeedforward(Feedforward.Right.kS, Feedforward.Right.kV, Feedforward.Right.kA);
  private DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Dimensions.trackWidthMeters);
  private DifferentialDrivePoseEstimator m_poseEstimator;
  private PIDController m_leftPIDs = new PIDController(PIDs.Left.kS, PIDs.Left.kV, PIDs.Left.kA);
  private PIDController m_rightPIDs = new PIDController(PIDs.Right.kS, PIDs.Right.kV, PIDs.Right.kA);

  //Create encoder and gyro objects
  private Encoder m_leftEncoder = new Encoder(Encoders.leftAPort, Encoders.leftBPort);
  private Encoder m_rightEncoder = new Encoder(Encoders.rightAPort, Encoders.rightBPort);
  private final AHRS m_gyro = new AHRS(Port.kMXP);

  //Create vision objects
  private PhotonCamera m_aprilTagCamera = new PhotonCamera("AprilTagCam");
  private AprilTagFieldLayout m_aprilTagFieldLayout;
  private List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
  private RobotPoseEstimator robotPoseEstimator;

  //Shuffleboard
  private ShuffleboardTab m_SBTab = Shuffleboard.getTab("Pose Estimation");
  private ShuffleboardLayout m_SBSensors;
  private Field2d m_robotField2d = new Field2d();

  //Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    //Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which group selected by Constants.Drive.kInvertDrive; left = False)
    m_left.setInverted(!Dimensions.kInvertDrive);
    m_right.setInverted(Dimensions.kInvertDrive);
    m_leftEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);
    m_rightEncoder.setDistancePerPulse(Dimensions.wheelCircumferenceMeters/Encoders.PPR);

    //TODO clean up this garbage
    try{
      m_aprilTagFieldLayout = new AprilTagFieldLayout(new File(Filesystem.getDeployDirectory(), "HallLayout.json").toPath());
    }
    catch(Exception e){
      System.out.println("Failed to load AprilTag Layout");
    }
    camList.add(new Pair<PhotonCamera, Transform3d>(m_aprilTagCamera, Vision.aprilTagCameraPositionTransform));
    robotPoseEstimator = new RobotPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.LOWEST_AMBIGUITY, camList);

    m_poseEstimator = new DifferentialDrivePoseEstimator(
      m_driveKinematics,
      new Rotation2d(getAngle()),
      getLeftDistance(), getRightDistance(),
      new Pose2d(3, 3, new Rotation2d(0)),
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
      new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.

    shuffleBoardInit();
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    m_leftLead.setIdleMode(nMode);
    m_leftFollow.setIdleMode(nMode);
    m_rightLead.setIdleMode(nMode);
    m_leftFollow.setIdleMode(nMode);
  }

  //Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent){
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed = m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed, Rate.maxSpeed)).omegaRadiansPerSecond;
    driveChassisSpeeds(new ChassisSpeeds(Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  //Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent){
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1 , 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(Rate.maxSpeed * leftPercent, Rate.maxSpeed * rightPercent));
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds + PIDs now
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    m_left.setVoltage( m_leftFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    m_right.setVoltage( m_rightFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){ driveWheelSpeeds(m_driveKinematics.toWheelSpeeds(chassisSpeeds));}

  //Drive the motors at a given voltage
  public void driveVoltages(double leftVoltage, double rightVoltage){
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
  }

  //PID Control
  public void FeedforwardPIDControl(DifferentialDriveWheelSpeeds wheelSpeeds, double leftVelocitySetpoint, double rightVelocitySetpoint){
    m_left.setVoltage(m_leftPIDs.calculate(m_leftEncoder.getRate(), leftVelocitySetpoint) + m_leftFeedforward.calculate(leftVelocitySetpoint));
    m_right.setVoltage(m_rightPIDs.calculate(m_rightEncoder.getRate(),rightVelocitySetpoint) + m_rightFeedforward.calculate(rightVelocitySetpoint));
  }
  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}

  //Encoder and gyro methods
  public double getLeftDistance(){ return m_leftEncoder.getDistance();}

  public double getRightDistance(){ return m_rightEncoder.getDistance();}

  public double getLeftVelocity(){ return m_rightEncoder.getRate();}

  public double getRightVelocity(){ return m_leftEncoder.getRate();}

  public double getAngle(){ return Units.degreesToRadians(-m_gyro.getYaw());}


  public Trajectory generateTrajectory(Pose2d endPose, ArrayList<Translation2d> waypoints) {

    //Starting Position
    var StartPosition = m_poseEstimator.getEstimatedPosition();
    //Desired Postion
    var EndPosition = endPose;
    //Empty list of waypoints
    var interiorWaypoints = waypoints;
    //Config
    TrajectoryConfig config = new TrajectoryConfig(Units.feetToMeters(TrajectoryConstants.kMaxSpeedMetersPerSecond), Units.feetToMeters(TrajectoryConstants.kMaxAccelerationMetersPerSecondSquared))
    .setKinematics(m_driveKinematics);
    config.setReversed(TrajectoryConstants.setReversed);
    //Initialize Traj
    var trajectory = TrajectoryGenerator.generateTrajectory(
        StartPosition,
        interiorWaypoints,
        EndPosition,
        config);
    return trajectory;
  }

  private void shuffleBoardInit(){
    m_SBSensors = m_SBTab.getLayout("Sensors", BuiltInLayouts.kList)
    .withSize(2,4)
    .withPosition(0, 0);
    m_SBSensors.add("NavX2", m_gyro).withWidget(BuiltInWidgets.kGyro);
    m_SBSensors.add("Left Encoder", m_leftEncoder).withWidget(BuiltInWidgets.kEncoder);
    m_SBSensors.add("Right Encoder", m_rightEncoder).withWidget(BuiltInWidgets.kEncoder);
    m_SBTab.add("Pose Estimate", m_robotField2d).withWidget(BuiltInWidgets.kField)
      .withSize(7, 4)
      .withPosition(2, 0);
  }

  @Override
  public void periodic() {
    //Update pose estimator with odometry
    m_poseEstimator.updateWithTime(
      Timer.getFPGATimestamp(),
      new Rotation2d(getAngle()),
      getLeftDistance(), 
      getRightDistance());
    m_robotField2d.setRobotPose(m_poseEstimator.getEstimatedPosition());

    //Get vision measurement and pass it to pose estimator
    robotPoseEstimator.setReferencePose(m_poseEstimator.getEstimatedPosition());
    Optional<Pair<Pose3d, Double>> result = robotPoseEstimator.update();
    if (result.isPresent()) {
      m_poseEstimator.addVisionMeasurement(
        result.get().getFirst().toPose2d(),
        Timer.getFPGATimestamp() - result.get().getSecond());
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}