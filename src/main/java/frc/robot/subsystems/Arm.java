package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kArm.Constraints;
import frc.robot.Constants.kArm.Dimensions;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax shoulderNeo1 =
      new CANSparkMax(CanId.shoulderNeo1, MotorType.kBrushless);
  private final CANSparkMax shoulderNeo2 =
      new CANSparkMax(CanId.shoulderNeo2, MotorType.kBrushless);
  private final CANSparkMax elbowNeo = new CANSparkMax(CanId.elbowNeo, MotorType.kBrushless);
  private final WPI_TalonSRX leftLead = new WPI_TalonSRX(CanId.turret);

  public Arm() {
    shoulderNeo2.follow(shoulderNeo1);
  }

  // Enable or disable brake mode on the motors
  public void brakeMode(boolean mode) {
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    shoulderNeo1.setIdleMode(nMode);
    shoulderNeo2.setIdleMode(nMode);
  }

  // TODO figure out how to set with current - Current PIDS
  public void setTurretCurrent() {}

  public void setShoulderCurrent() {}

  public void setElbowCurrent() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Matrix<N2, N1> kinematics(Matrix<N2, N1> matrixSE) {
    double xG =
        Dimensions.Lp * Math.cos(matrixSE.get(0, 0)) + Dimensions.Lf * Math.cos(matrixSE.get(1, 0));
    double yG =
        Dimensions.Lp * Math.sin(matrixSE.get(0, 0)) + Dimensions.Lf * Math.sin(matrixSE.get(1, 0));
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(xG, yG);
  }

  public Matrix<N2, N1> inverseKinematics(Matrix<N2, N1> matrixXY) {
    double r = Math.sqrt(Math.pow(matrixXY.get(0, 0), 2) + Math.pow(matrixXY.get(1, 0), 2));
    double theta_s =
        Math.atan(matrixXY.get(1, 0) / matrixXY.get(0, 0))
            + Math.acos(
                (Math.pow(r, 2) + Math.pow(Dimensions.Lp, 2) - Math.pow(Dimensions.Lf, 2))
                    / (2 * r * Dimensions.Lp));
    double theta_e =
        Math.atan(matrixXY.get(1, 0) / matrixXY.get(0, 0))
            - Math.acos(
                (Math.pow(r, 2) + Math.pow(Dimensions.Lf, 2) - Math.pow(Dimensions.Lp, 2))
                    / (2 * r * Dimensions.Lf));
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
}
