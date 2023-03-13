package frc.robot.controls;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.kArm.*;

public class DoubleJointedArmController {
  private final double Kt = Motors.stall_torque / Motors.stall_current;
  private final double Kv = Motors.free_speed / 12; // 12 volt peak power
  private final double R = 12 / Motors.stall_current;
  private final double g = 9.81; // g is 9.81 m/s^2
  private final Matrix<N2, N2> Kb =
      new MatBuilder<>(Nat.N2(), Nat.N2())
          .fill(
              (Math.pow(Proximal.gear_ratio, 2) * Proximal.num_motors * Kt) / (Kv * R),
              0,
              0,
              (Math.pow(Forearm.gear_ratio, 2) * Forearm.num_motors * Kt) / (Kv * R));
  private final Matrix<N2, N2> B =
      new MatBuilder<>(Nat.N2(), Nat.N2())
          .fill(
              (Proximal.gear_ratio * Proximal.num_motors * Kt) / R,
              0,
              0,
              (Forearm.gear_ratio * Forearm.num_motors * Kt) / R);

  private final PlantInversionFeedForwardOnCrack<N4, N2> feedforward;
  private final PIDController proximalPID;
  private final PIDController forearmPID;

  public DoubleJointedArmController(
      double proximal_kP, double proximal_kD, double forearm_kP, double forearm_kD) {
    this.feedforward =
        new PlantInversionFeedForwardOnCrack<>(
            Nat.N4(), Nat.N2(), this::system_model, 20.0 / 1000.0);
    this.proximalPID = new PIDController(proximal_kP, 0, proximal_kD);
    this.forearmPID = new PIDController(forearm_kP, 0, forearm_kD);
  }

  private double dis_pulley(double theta) {
    return Math.sqrt(
        Math.pow(Proximal.len_anchor, 2)
            + Math.pow(Proximal.len_pulley, 2)
            - 2
                * Proximal.len_anchor
                * Proximal.len_pulley
                * Math.cos(Proximal.angle_anchor - (theta + Proximal.angle_pulley)));
  }

  private double c1(Matrix<N2, N1> joints) {
    return Math.cos(joints.get(0, 0));
  }

  private double c2(Matrix<N2, N1> joints) {
    return Math.cos(joints.get(1, 0));
  }

  private double c12(Matrix<N2, N1> joints) {
    return Math.cos(joints.get(0, 0) + joints.get(1, 0));
  }

  private double s2(Matrix<N2, N1> joints) {
    return Math.sin(joints.get(1, 0));
  }

  // Inertia matrix
  private Matrix<N2, N2> M(Matrix<N4, N1> x) {
    var angles = x.block(Nat.N2(), Nat.N1(), 0, 0);
    return new MatBuilder<>(Nat.N2(), Nat.N2())
        .fill(
            Proximal.mass * Math.pow(Proximal.radius, 2)
                + Forearm.mass * (Math.pow(Proximal.length, 2) + Math.pow(Forearm.radius, 2))
                + Proximal.inertia
                + Forearm.inertia
                + 2 * Forearm.mass * Proximal.length * Forearm.radius * c2(angles),
            Forearm.mass * Math.pow(Forearm.radius, 2)
                + Forearm.inertia
                + Forearm.mass * Proximal.length * Forearm.radius * c2(angles),
            Forearm.mass * Math.pow(Forearm.radius, 2)
                + Forearm.inertia
                + Forearm.mass * Proximal.length * Forearm.radius * c2(angles),
            Forearm.mass * Math.pow(Forearm.radius, 2) + Forearm.inertia);
  }

  // Coriolis matrix
  private Matrix<N2, N2> C(Matrix<N4, N1> x) {
    return new MatBuilder<>(Nat.N2(), Nat.N2())
        .fill(
            -Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(x.block(Nat.N2(), Nat.N1(), 0, 0))
                * x.get(3, 0),
            -Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(x.block(Nat.N2(), Nat.N1(), 0, 0))
                * (x.get(2, 0) + x.get(3, 0)),
            Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(x.block(Nat.N2(), Nat.N1(), 0, 0))
                * x.get(0, 0),
            0);
  }
  // Gravity matrix
  private Matrix<N2, N1> Tg(Matrix<N4, N1> x) {
    var angles = x.block(Nat.N2(), Nat.N1(), 0, 0);
    return new MatBuilder<>(Nat.N2(), Nat.N1())
        .fill(
            (Proximal.mass * Proximal.radius + Forearm.mass * Proximal.length) * g * c1(angles)
                + Forearm.mass * Forearm.radius * g * c12(angles),
            Forearm.mass * Forearm.radius * g * c12(angles));
  }

  private Matrix<N2, N1> Tsp(Matrix<N4, N1> x) {
    var angles = x.block(Nat.N2(), Nat.N1(), 0, 0);
    if (angles.get(0, 0) >= Math.PI / 2) {
      return new MatBuilder<>(Nat.N2(), Nat.N1())
          .fill(0, 0);
    } else {
      return new MatBuilder<>(Nat.N2(), Nat.N1())
          .fill(
              Proximal.k_spring
                  * (dis_pulley(angles.get(0, 0) - dis_pulley(Math.PI / 2)))
                  * (Proximal.len_anchor
                      * Math.sin(
                          Proximal.angle_anchor - (angles.get(0, 0) + Proximal.angle_pulley)))
                  / dis_pulley(angles.get(0, 0))
                  * Proximal.len_pulley,
              0);
    }
  }

  public Matrix<N4, N1> system_model(Matrix<N4, N1> x, Matrix<N2, N1> u) {
    Matrix<N2, N1> omegaVector = x.block(Nat.N2(), Nat.N1(), 2, 0);

    Matrix<N2, N1> torqueMatrix =
        B.times(u)
            .minus(Kb.times(omegaVector))
            .minus(C(x).times(omegaVector))
            .minus(Tg(x))
            .plus(Tsp(x));

    Matrix<N2, N1> alphaVector = M(x).inv().times(torqueMatrix);

    Matrix<N4, N1> x_dot = new Matrix<N4, N1>(Nat.N4(), Nat.N1());
    x_dot.assignBlock(0, 0, omegaVector);
    x_dot.assignBlock(2, 0, alphaVector);

    return x_dot;
  }

  public void reset(Matrix<N4, N1> x) {
    feedforward.reset(x);
    proximalPID.reset();
    forearmPID.reset();
  }

  public Matrix<N2, N1> calculate(Matrix<N4, N1> measurement, Matrix<N4, N1> nextR) {
    var FF_result = feedforward.calculate(nextR);
    var PID_result =
        new MatBuilder<>(Nat.N2(), Nat.N1())
            .fill(
                proximalPID.calculate(measurement.get(0, 0), nextR.get(0, 0)),
                forearmPID.calculate(measurement.get(1, 0), nextR.get(1, 0)));

    var combined = FF_result.plus(PID_result);
    var clamped =
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1())
            .fill(
                MathUtil.clamp(combined.get(0, 0), -10.5, 10.5),
                MathUtil.clamp(combined.get(1, 0), -10.5, 10.5));
    return clamped;
  }

  public Matrix<N2, N1> calculate(
      Matrix<N4, N1> measurement, Matrix<N4, N1> r, Matrix<N4, N1> nextR) {
    var FF_result = feedforward.calculate(r, nextR);
    var PID_result =
        new MatBuilder<>(Nat.N2(), Nat.N1())
            .fill(
                proximalPID.calculate(measurement.get(0, 0), nextR.get(0, 0)),
                forearmPID.calculate(measurement.get(1, 0), nextR.get(1, 0)));

    return FF_result.plus(PID_result);
  }
}
