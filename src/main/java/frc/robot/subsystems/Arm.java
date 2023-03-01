package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.system.NumericalIntegration;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kArm.*;
import frc.robot.Robot;
import frc.robot.commands.ArmTrajectoryCommand;
import frc.robot.controls.DoubleJointedArmController;
import frc.robot.utilities.ArmTrajectory;
import java.util.Map;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax proximalNEO1 =
      new CANSparkMax(CanId.proximalNEO1, MotorType.kBrushless);
  private final CANSparkMax proximalNEO2 =
      new CANSparkMax(CanId.proximalNEO2, MotorType.kBrushless);
  private final CANSparkMax forearmNEO = new CANSparkMax(CanId.forearmNEO, MotorType.kBrushless);
  private final WPI_TalonSRX turretController = new WPI_TalonSRX(CanId.turret);

  // Encoder Objects
  private final DutyCycleEncoder absProximalEncoder =
      new DutyCycleEncoder(Encoders.Proximal.absPort);
  private final DutyCycleEncoder absForearmEncoder = new DutyCycleEncoder(Encoders.Forearm.absPort);
  private final DutyCycleEncoder absTurretEncoder = new DutyCycleEncoder(Encoders.Turret.absPort);

  private final Encoder proximalEncoder =
      new Encoder(Encoders.Proximal.APort, Encoders.Proximal.BPort);
  private final Encoder forearmEncoder =
      new Encoder(Encoders.Forearm.APort, Encoders.Forearm.BPort);
  private final Encoder turretEncoder = new Encoder(Encoders.Turret.APort, Encoders.Turret.BPort);

  private EncoderSim proximalEncoderSim = new EncoderSim(proximalEncoder);
  private EncoderSim forearmEncoderSim = new EncoderSim(forearmEncoder);
  private EncoderSim turretEncoderSim = new EncoderSim(turretEncoder);

  // Offsets and states
  private double proximalOffset = -100;
  private double forearmOffset = -100;
  private double turretOffset = -100;

  private final SimpleMotorFeedforward TurretFeedforward =
      new SimpleMotorFeedforward(Turret.ks, Turret.kv, Turret.ka);
  private final PIDController TurretPID = new PIDController(Turret.kp, Turret.ki, Turret.kd);

  private Matrix<N4, N1> armSetpoint;
  private Matrix<N4, N1> simState;

  // Controls objects
  private final DoubleJointedArmController armController;

  // Mechanism 2d
  private Mechanism2d arm2d = new Mechanism2d(3, 2.2);
  private MechanismRoot2d base2d = arm2d.getRoot("Arm", 1, 0.2413);
  private MechanismLigament2d proximal2d =
      base2d.append(
          new MechanismLigament2d(
              "Proximal", Proximal.length, 60, 5, new Color8Bit(255, 255, 255)));
  private MechanismLigament2d forearm2d =
      proximal2d.append(
          new MechanismLigament2d("Forearm", Forearm.length, -30, 5, new Color8Bit(255, 255, 255)));
  private MechanismLigament2d gripper2d =
      forearm2d.append(new MechanismLigament2d("Gripper", .25, -30, 5, new Color8Bit(0, 255, 0)));

  // Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Arm");
  private ShuffleboardLayout SBSensors;
  private ShuffleboardLayout SBMotors;
  private ShuffleboardLayout SBState;

  // Logging
  private DataLog log = DataLogManager.getLog();
  private DoubleArrayLogEntry logActualVoltages =
      new DoubleArrayLogEntry(log, "/ArmSubsystem/actualVoltages");
  private DoubleArrayLogEntry logAbsoluteEncoderValues =
      new DoubleArrayLogEntry(log, "/ArmSubsystem/absoluteEncoderValues");
  private DoubleArrayLogEntry logEncoderOffsets =
      new DoubleArrayLogEntry(log, "/ArmSubsystem/encoderOffsets");
  private DoubleArrayLogEntry logState = new DoubleArrayLogEntry(log, "/ArmSubsystem/state");
  private DoubleArrayLogEntry logSetpoint = new DoubleArrayLogEntry(log, "/ArmSubsystem/setpoint");
  private DoubleArrayLogEntry logCalculatedVoltages =
      new DoubleArrayLogEntry(log, "/ArmSubsystem/calculatedVoltages");

  public Arm() {
    proximalNEO1.setIdleMode(IdleMode.kBrake);
    proximalNEO1.setInverted(true);
    proximalNEO2.setIdleMode(IdleMode.kBrake);
    proximalNEO2.follow(proximalNEO1, true);

    forearmNEO.setIdleMode(IdleMode.kBrake);
    forearmNEO.setInverted(true);
    turretController.setNeutralMode(NeutralMode.Brake);

    absProximalEncoder.setDistancePerRotation(-2 * Math.PI * Encoders.Proximal.gear_ratio);
    absForearmEncoder.setDistancePerRotation(2 * Math.PI * Encoders.Forearm.gear_ratio);
    absTurretEncoder.setDistancePerRotation(2 * Math.PI * Encoders.Turret.gear_ratio);

    proximalEncoder.setDistancePerPulse(2 * Math.PI * Encoders.Proximal.gear_ratio / Encoders.PPR);
    forearmEncoder.setDistancePerPulse(2 * Math.PI * Encoders.Forearm.gear_ratio / Encoders.PPR);
    forearmEncoder.setReverseDirection(true);
    turretEncoder.setDistancePerPulse(2 * Math.PI * Encoders.Turret.gear_ratio / Encoders.PPR);
    turretEncoder.setReverseDirection(true);

    if (!Robot.isReal()) {
      proximalOffset = Encoders.Proximal.initial;
      forearmOffset = Encoders.Forearm.initial;
      turretOffset = Encoders.Turret.initial;
    }

    proximalEncoder.reset();
    forearmEncoder.reset();
    turretEncoder.reset();

    new WaitUntilCommand(
            () ->
                absProximalEncoder.isConnected()
                    && absForearmEncoder.isConnected()
                    && absTurretEncoder.isConnected())
        .withTimeout(5)
        .andThen(new WaitCommand(1))
        .andThen(this::encoderInit)
        .schedule();

    armSetpoint = getArmMeasuredStates();
    simState = armSetpoint;

    armController =
        new DoubleJointedArmController(
            Feedback.proximal_kP, Feedback.proximal_kD, Feedback.forearm_kP, Feedback.forearm_kD);
    reset(armSetpoint);

    telemetryInit();
  }

  public Matrix<N2, N1> kinematics2D(Matrix<N2, N1> matrixSE) {
    double xG =
        Proximal.length * Math.cos(matrixSE.get(0, 0))
            + Forearm.length * Math.cos(matrixSE.get(0, 0) + matrixSE.get(1, 0));
    double yG =
        Proximal.length * Math.sin(matrixSE.get(0, 0))
            + Forearm.length * Math.sin(matrixSE.get(0, 0) + matrixSE.get(1, 0));
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(xG, yG);
  }

  public Matrix<N3, N1> kinematics3D(Matrix<N3, N1> matrixSETurret) {
    double xG =
        (Proximal.length * Math.cos(matrixSETurret.get(0, 0))
                + Forearm.length * Math.cos(matrixSETurret.get(1, 0)))
            * Math.cos(matrixSETurret.get(2, 0));
    double yG =
        (Proximal.length * Math.cos(matrixSETurret.get(0, 0))
                + Forearm.length * Math.cos(matrixSETurret.get(1, 0)))
            * Math.sin(matrixSETurret.get(2, 0));
    double zG =
        Proximal.length * Math.sin(matrixSETurret.get(0, 0))
            + Forearm.length * Math.sin(matrixSETurret.get(1, 0));
    return new MatBuilder<>(Nat.N3(), Nat.N1()).fill(xG, yG, zG);
  }

  public Matrix<N2, N1> inverseKinematics(Matrix<N2, N1> matrixXY) {
    double r = Math.sqrt(Math.pow(matrixXY.get(0, 0), 2) + Math.pow(matrixXY.get(1, 0), 2));
    double theta_s =
        Math.atan(matrixXY.get(1, 0) / matrixXY.get(0, 0))
            + Math.acos(
                (Math.pow(r, 2) + Math.pow(Proximal.length, 2) - Math.pow(Forearm.length, 2))
                    / (2 * r * Proximal.length));
    double theta_e =
        Math.atan(matrixXY.get(1, 0) / matrixXY.get(0, 0))
            - Math.acos(
                (Math.pow(r, 2) + Math.pow(Forearm.length, 2) - Math.pow(Proximal.length, 2))
                    / (2 * r * Forearm.length));
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(theta_s, theta_e - theta_s);
  }

  public Pair<TrapezoidProfile, TrapezoidProfile> motionProfile(
      Matrix<N2, N1> startXY, Matrix<N2, N1> endXY) {
    // Inverse Kinematics to get the Thetas
    Matrix<N2, N1> initialThetas = inverseKinematics(startXY); // Shoulder, then Elbow
    Matrix<N2, N1> endThetas = inverseKinematics(endXY);
    // Create the Motion Profiles
    TrapezoidProfile profileShoulder =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constraints.Velocity, Constraints.Acceleration), // contraints
            new TrapezoidProfile.State(endThetas.get(0, 0), 0), // endpoint
            new TrapezoidProfile.State(initialThetas.get(0, 0), 0)); // startpoint
    TrapezoidProfile profileElbow =
        new TrapezoidProfile(
            new TrapezoidProfile.Constraints(
                Constraints.Velocity, Constraints.Acceleration), // contraints
            new TrapezoidProfile.State(endThetas.get(1, 0), 0), // endpoint
            new TrapezoidProfile.State(initialThetas.get(1, 0), 0)); // startpoint
    return new Pair<>(profileShoulder, profileElbow); // Return as pair
  }

  private Matrix<N4, N1> getArmMeasuredStates() {
    return new MatBuilder<N4, N1>(Nat.N4(), Nat.N1())
        .fill(
            proximalEncoder.getDistance() + proximalOffset,
            forearmEncoder.getDistance() + forearmOffset,
            proximalEncoder.getRate(),
            forearmEncoder.getRate());
  }

  private void encoderInit() {
    proximalOffset =
        absProximalEncoder.getDistance() + Encoders.Proximal.initial - Encoders.Proximal.offset;
    forearmOffset =
        absForearmEncoder.getDistance() + Encoders.Forearm.initial - Encoders.Forearm.offset;
    turretOffset =
        absTurretEncoder.getDistance() + Encoders.Turret.initial - Encoders.Turret.offset;

    setArmSetpoint(getArmMeasuredStates());

    logAbsoluteEncoderValues.append(
        new double[] {
          absProximalEncoder.getDistance(),
          absForearmEncoder.getDistance(),
          absTurretEncoder.getDistance()
        });
    logEncoderOffsets.append(new double[] {proximalOffset, forearmOffset, turretOffset});
  }

  public void setVoltages(double shoulder, double elbow) {
    proximalNEO1.setVoltage(shoulder);
    forearmNEO.setVoltage(elbow);
  }

  private void setArmVoltages(Matrix<N2, N1> voltages) {
    proximalNEO1.setVoltage(voltages.get(0, 0));
    forearmNEO.setVoltage(voltages.get(1, 0));
  }

  public void setTurretVoltages(double setpoint) {
    turretController.setVoltage(
        TurretFeedforward.calculate(setpoint)
            + TurretPID.calculate(turretEncoder.getRate(), setpoint));
  }

  private Matrix<N2, N1> getArmVoltages() {
    return new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
        .fill(proximalNEO1.getAppliedOutput(), forearmNEO.getAppliedOutput());
  }

  public void telemetryInit() {
    SmartDashboard.putData("Arm", arm2d);
    SBMotors = SBTab.getLayout("Motors", BuiltInLayouts.kList).withSize(2, 4).withPosition(0, 0);
    SBSensors =
        SBTab.getLayout("Sensors", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2))
            .withSize(5, 4)
            .withPosition(2, 0);
    SBState =
        SBTab.getLayout("State", BuiltInLayouts.kGrid)
            .withProperties(Map.of("Number of columns", 2))
            .withSize(5, 4)
            .withPosition(7, 0);

    SBMotors.addDouble("Shoulder Output", () -> getArmVoltages().get(0, 0));
    SBMotors.addDouble("Elbow Output", () -> getArmVoltages().get(1, 0));

    SBSensors.add("Proximal Quad Encoder", proximalEncoder)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(0, 0);
    SBSensors.add("Forearm Quad Encoder", forearmEncoder)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(0, 1);
    SBSensors.add("Turret Quad Encoder", turretEncoder)
        .withWidget(BuiltInWidgets.kEncoder)
        .withPosition(0, 2);
    SBSensors.add("Proximal Absolute", absProximalEncoder).withPosition(1, 0);
    SBSensors.add("Forearm Absolute", absForearmEncoder).withPosition(1, 1);
    SBSensors.add("Turret Absolute", absTurretEncoder).withPosition(1, 2);

    SBState.addDouble("Proximal", () -> proximalEncoder.getDistance() + proximalOffset)
        .withPosition(0, 0);
    SBState.addDouble("Forearm", () -> forearmEncoder.getDistance() + forearmOffset)
        .withPosition(0, 1);
    SBState.addDouble("Turret", () -> turretEncoder.getDistance() + turretOffset)
        .withPosition(0, 2);
    SBState.addDouble("X", () -> kinematics2D(getArmMeasuredStates().block(2, 1, 0, 0)).get(0, 0))
        .withPosition(1, 0);
    SBState.addDouble("Y", () -> kinematics2D(getArmMeasuredStates().block(2, 1, 0, 0)).get(1, 0))
        .withPosition(1, 1);
  }

  public void setArmSetpoint(Matrix<N4, N1> setpoint) {
    armSetpoint = setpoint;
  }

  public void reset(Matrix<N4, N1> state) {
    armController.reset(state);
  }

  public Command simpleTrajectory(double startx, double starty, double endx, double endy) {
    var profile =
        motionProfile(
            new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(startx, starty),
            new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(endx, endy));
    return new ArmTrajectoryCommand(new ArmTrajectory(profile), this);
  }

  @Override
  public void periodic() {
    // Check for encoder init

    // Calculate voltages
    var state = getArmMeasuredStates();
    var voltages = armController.calculate(state, armSetpoint);
    setArmVoltages(voltages);

    // Log values and update mechanism2d
    logSetpoint.append(
        new double[] {
          armSetpoint.get(0, 0), armSetpoint.get(1, 0), armSetpoint.get(2, 0), armSetpoint.get(3, 0)
        });
    logState.append(
        new double[] {state.get(0, 0), state.get(1, 0), state.get(2, 0), state.get(3, 0)});
    logCalculatedVoltages.append(new double[] {voltages.get(0, 0), voltages.get(1, 0)});
    logActualVoltages.append(
        new double[] {proximalNEO1.getAppliedOutput(), forearmNEO.getAppliedOutput()});
    proximal2d.setAngle(Units.radiansToDegrees(getArmMeasuredStates().get(0, 0)));
    forearm2d.setAngle(Units.radiansToDegrees(getArmMeasuredStates().get(1, 0)));
    gripper2d.setAngle(
        Units.radiansToDegrees(
            -(getArmMeasuredStates().get(0, 0) + getArmMeasuredStates().get(1, 0))));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    simState =
        NumericalIntegration.rkdp(
            armController::system_model, simState, getArmVoltages(), 20.0 / 1000.0);
    proximalEncoderSim.setDistance(simState.get(0, 0) - proximalOffset);
    forearmEncoderSim.setDistance(simState.get(1, 0) - forearmOffset);
    proximalEncoderSim.setRate(simState.get(2, 0));
    forearmEncoderSim.setRate(simState.get(3, 0));
  }
}
