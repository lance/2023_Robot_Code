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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kArm.*;
import frc.robot.commands.ArmTrajectoryCommand;
import frc.robot.controls.DoubleJointedArmController;
import frc.robot.utilities.ArmTrajectory;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax proximalNEO1 =
      new CANSparkMax(CanId.proximalNEO1, MotorType.kBrushless);
  private final CANSparkMax proximalNEO2 =
      new CANSparkMax(CanId.proximalNEO2, MotorType.kBrushless);
  private final CANSparkMax forearmNEO = new CANSparkMax(CanId.forearmNEO, MotorType.kBrushless);
  private final WPI_TalonSRX turretController = new WPI_TalonSRX(CanId.turret);

  private final DutyCycleEncoder absProximalEncoder =
      new DutyCycleEncoder(Encoders.Proximal.absPort);
  private final DutyCycleEncoder absForearmEncoder = new DutyCycleEncoder(Encoders.Forearm.absPort);
  private final DutyCycleEncoder absTurretEncoder = new DutyCycleEncoder(Encoders.Turret.absPort);

  private final Encoder proximalEncoder =
      new Encoder(Encoders.Proximal.APort, Encoders.Proximal.BPort);
  private final Encoder forearmEncoder =
      new Encoder(Encoders.Forearm.APort, Encoders.Forearm.BPort);
  private final Encoder turretEncoder = new Encoder(Encoders.Turret.APort, Encoders.Turret.BPort);

  private double proximalOffset;
  private double forearmOffset;
  private double turretOffset;

  private final SimpleMotorFeedforward TurretFeedforward =
      new SimpleMotorFeedforward(Turret.ks, Turret.kv, Turret.ka);
  private final PIDController TurretPID = new PIDController(Turret.kp, Turret.ki, Turret.kd);

  private Matrix<N4, N1> armSetpoint;

  private final DoubleJointedArmController armController;

  private Mechanism2d arm2d = new Mechanism2d(4, 4);
  private MechanismRoot2d base2d = arm2d.getRoot("Arm", .5, 0);
  private MechanismLigament2d proximal2d =
      base2d.append(new MechanismLigament2d("Proximal", 1.5, 60, 5, new Color8Bit(255, 255, 255)));
  private MechanismLigament2d forearm2d =
      proximal2d.append(
          new MechanismLigament2d("Forearm", 1.25, -30, 3, new Color8Bit(255, 255, 255)));
  private MechanismLigament2d gripper2d =
      forearm2d.append(new MechanismLigament2d("Gripper", .4, -30, 2, new Color8Bit(0, 255, 0)));

  private EncoderSim proximalEncoderSim = new EncoderSim(proximalEncoder);
  private EncoderSim forearmEncoderSim = new EncoderSim(forearmEncoder);
  private EncoderSim turretEncoderSim = new EncoderSim(turretEncoder);

  // Shuffleboard
  private ShuffleboardTab SBTab = Shuffleboard.getTab("Arm");
  Matrix x = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.75, -1.5, 0, 0);

  public Arm() {
    proximalNEO1.setIdleMode(IdleMode.kBrake);
    proximalNEO2.setIdleMode(IdleMode.kBrake);
    proximalNEO2.follow(proximalNEO1);

    forearmNEO.setIdleMode(IdleMode.kBrake);
    turretController.setNeutralMode(NeutralMode.Brake);

    absProximalEncoder.setDistancePerRotation(Math.PI * Encoders.Proximal.gear_ratio);
    absForearmEncoder.setDistancePerRotation(Math.PI * Encoders.Forearm.gear_ratio);
    absTurretEncoder.setDistancePerRotation(Math.PI * Encoders.Turret.gear_ratio);

    proximalEncoder.setDistancePerPulse(Math.PI * Encoders.Proximal.gear_ratio / Encoders.PPR);
    forearmEncoder.setDistancePerPulse(Math.PI * Encoders.Forearm.gear_ratio / Encoders.PPR);
    turretEncoder.setDistancePerPulse(Math.PI * Encoders.Turret.gear_ratio / Encoders.PPR);

    proximalOffset = absProximalEncoder.getDistance() + Encoders.Proximal.initial;
    forearmOffset = absForearmEncoder.getDistance() + Encoders.Forearm.initial;
    turretOffset = absTurretEncoder.getDistance() + Encoders.Turret.initial;

    proximalEncoder.reset();
    forearmEncoder.reset();
    turretEncoder.reset();

    armSetpoint = getArmMeasuredStates();
    armSetpoint.assignBlock(
        0, 0, inverseKinematics(new MatBuilder(Nat.N2(), Nat.N1()).fill(1, 0.1)));

    armController =
        new DoubleJointedArmController(
            Feedback.proximal_kP, Feedback.proximal_kD, Feedback.forearm_kP, Feedback.forearm_kD);

    shuffleBoardInit();
  }

  public Matrix<N2, N1> kinematics2D(Matrix<N2, N1> matrixSE) {
    double xG =
        Proximal.length * Math.cos(matrixSE.get(0, 0))
            + Forearm.length * Math.cos(matrixSE.get(0, 0) - matrixSE.get(1, 0));
    double yG =
        Proximal.length * Math.sin(matrixSE.get(0, 0))
            + Forearm.length * Math.sin(matrixSE.get(0, 0) - matrixSE.get(1, 0));
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

  private void setArmVoltages(Matrix<N2, N1> voltages) {
    proximalNEO1.setVoltage(voltages.get(0, 0));
    forearmNEO.setVoltage(voltages.get(1, 0));
  }

  public void setTurretVoltages(double setpoint) {
    turretController.setVoltage(
        TurretFeedforward.calculate(setpoint)
            + TurretPID.calculate(turretEncoder.getRate(), setpoint));
  }

  public void shuffleBoardInit() {
    SBTab.add("Arm 2d", arm2d);
  }

  public void setArmSetpoint(Matrix<N4, N1> setpoint) {
    armSetpoint = setpoint;
  }

  public Command simpleTrajectory(double startx, double starty, double endx, double endy) {
    var profile =
        motionProfile(
            new MatBuilder(Nat.N2(), Nat.N1()).fill(startx, starty),
            new MatBuilder(Nat.N2(), Nat.N1()).fill(endx, endy));
    return new ArmTrajectoryCommand(new ArmTrajectory(profile), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setArmVoltages(armController.calculate(getArmMeasuredStates(), armSetpoint));
    proximal2d.setAngle(Units.radiansToDegrees(getArmMeasuredStates().get(0, 0)));
    forearm2d.setAngle(Units.radiansToDegrees(getArmMeasuredStates().get(1, 0)));
    gripper2d.setAngle(
        Units.radiansToDegrees(
            -(getArmMeasuredStates().get(0, 0) + getArmMeasuredStates().get(1, 0))));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    var next_states =
        NumericalIntegration.rkdp(
            armController::system_model,
            getArmMeasuredStates(),
            armController.calculate(getArmMeasuredStates(), armSetpoint),
            20.0 / 1000.0);
    proximalEncoderSim.setDistance(next_states.get(0, 0));
    forearmEncoderSim.setDistance(next_states.get(1, 0));
    proximalEncoderSim.setRate(next_states.get(2, 0));
    forearmEncoderSim.setRate(next_states.get(3, 0));
  }
}
