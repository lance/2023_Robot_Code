package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kArm.*;
import frc.robot.utilities.DoubleJointedArmController;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax proximalNEO1 =
      new CANSparkMax(CanId.proximalNEO1, MotorType.kBrushless);
  private final CANSparkMax proximalNEO2 =
      new CANSparkMax(CanId.proximalNEO2, MotorType.kBrushless);
  private final CANSparkMax forearmNEO = new CANSparkMax(CanId.forearmNEO, MotorType.kBrushless);
  private final TalonSRX turret = new TalonSRX(CanId.turret);

  private final DutyCycleEncoder absProximalEncoder =
      new DutyCycleEncoder(Encoders.Proximal.absPort);
  private final DutyCycleEncoder absForearmEncoder = new DutyCycleEncoder(Encoders.Forearm.absPort);
  private final DutyCycleEncoder absTurretEncoder = new DutyCycleEncoder(Encoders.Turret.absPort);

  private final Encoder proximalEncoder =
      new Encoder(Encoders.Proximal.APort, Encoders.Proximal.BPort);
  private final Encoder forearmEncoder =
      new Encoder(Encoders.Forearm.APort, Encoders.Forearm.BPort);
  private final Encoder turretEncoder = new Encoder(Encoders.Forearm.APort, Encoders.Forearm.BPort);

  private double proximalOffset;
  private double forearmOffset;
  private double turretOffset;

  private final Matrix<N4, N1> setpoint;

  private final DoubleJointedArmController armController;

  public Arm() {
    proximalNEO1.setIdleMode(IdleMode.kBrake);
    proximalNEO2.setIdleMode(IdleMode.kBrake);
    proximalNEO2.follow(proximalNEO1);

    forearmNEO.setIdleMode(IdleMode.kBrake);
    turret.setNeutralMode(NeutralMode.Brake);

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

    setpoint = getArmMeasuredStates();

    armController =
        new DoubleJointedArmController(
            Feedback.proximal_kP, Feedback.proximal_kD, Feedback.forearm_kP, Feedback.forearm_kD);
  }

  public Matrix<N3, N1> kinematics(Matrix<N3, N1> matrixSETurret) {
    double xG =
        (Proximal.length * Math.cos(matrixSETurret.get(0, 0))
            + Forearm.length * Math.cos(matrixSETurret.get(1, 0)))*Math.cos(matrixSETurret.get(2,0));
    double yG = (Proximal.length * Math.cos(matrixSETurret.get(0, 0))
            + Forearm.length * Math.cos(matrixSETurret.get(1, 0)))*Math.sin(matrixSETurret.get(2,0));
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
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(theta_s, theta_e);
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

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    setArmVoltages(armController.calculate(getArmMeasuredStates(), setpoint));
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
