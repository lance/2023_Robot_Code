package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Constants.kArm.*;
import java.util.function.Supplier;

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
              (Proximal.gear_ratio * Proximal.num_motors * Kt) / (Kv * R),
              0,
              0,
              (Forearm.gear_ratio * Forearm.num_motors * Kt) / (Kv * R));

  private final double proximal_kP;
  private final double proximal_kD;
  private final double forearm_kP;
  private final double forearm_kD;

  private final Supplier<Matrix<N4, N1>> state_supplier;

  public DoubleJointedArmController(
      double proximal_kP,
      double proximal_kD,
      double forearm_kP,
      double forearm_kD,
      Supplier<Matrix<N4, N1>> state_supplier) {
    this.proximal_kP = proximal_kP;
    this.proximal_kD = proximal_kD;
    this.forearm_kP = forearm_kP;
    this.forearm_kD = forearm_kD;
    this.state_supplier = state_supplier;
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

  private double s1(Matrix<N2, N1> joints) {
    return Math.sin(joints.get(0, 0));
  }

  private double s2(Matrix<N2, N1> joints) {
    return Math.sin(joints.get(1, 0));
  }

  private double s12(Matrix<N2, N1> joints) {
    return Math.sin(joints.get(0, 0) + joints.get(1, 0));
  }

  // Inertia matrix takes only joint angles
  private Matrix<N2, N2> M(Matrix<N2, N1> joints) {
    return new MatBuilder<>(Nat.N2(), Nat.N2())
        .fill(
            Proximal.mass * Math.pow(Proximal.radius, 2)
                + Forearm.mass * (Math.pow(Proximal.inertia, 2) + Math.pow(Forearm.radius, 2))
                + Proximal.inertia
                + Forearm.inertia
                + 2 * Forearm.mass * Forearm.inertia * Forearm.radius * c2(joints),
            Forearm.mass * Math.pow(Forearm.radius, 2)
                + Forearm.inertia
                + Forearm.mass * Proximal.length * Forearm.radius * c2(joints),
            Forearm.mass * Math.pow(Forearm.radius, 2)
                + Forearm.inertia
                + Forearm.mass * Proximal.length * Forearm.radius * c2(joints),
            Forearm.mass * Math.pow(Forearm.radius, 2) + Forearm.inertia);
  }

  // Coriolis matrix takes angles and velocities
  private Matrix<N2, N2> C(Matrix<N4, N1> joints) {
    return new MatBuilder<>(Nat.N2(), Nat.N2())
        .fill(
            -Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(joints.block(2, 1, 0, 0))
                * joints.get(3, 0),
            -Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(joints.block(2, 1, 0, 0))
                * (joints.get(2, 0) + joints.get(3, 0)),
            Forearm.mass
                * Proximal.length
                * Forearm.radius
                * s2(joints.block(2, 1, 0, 0))
                * joints.get(0, 0),
            0);
  }
  // Gravity matrix takes joint angles
  private Matrix<N2, N1> Tg(Matrix<N2, N1> joints) {
    return new MatBuilder<>(Nat.N2(), Nat.N1())
        .fill(
            (Proximal.mass * Proximal.radius + Forearm.mass * Proximal.length) * g * c1(joints)
                + Forearm.mass * Forearm.radius * g * c12(joints),
            Forearm.mass * Forearm.radius * g * c12(joints));
  }

  private Matrix<N2, N1> Tsp(Matrix<N2, N1> joints) {
    if (joints.get(0, 0) >= Math.PI / 2) {
      return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(0, Forearm.torque_spring);
    } else {
      return new MatBuilder<>(Nat.N2(), Nat.N1())
          .fill(
              Proximal.k_spring
                  * (dis_pulley(joints.get(0, 0) - dis_pulley(Math.PI / 2)))
                  * (Proximal.len_anchor
                      * Math.sin(
                          Proximal.angle_anchor - (joints.get(0, 0) + Proximal.angle_pulley)))
                  / dis_pulley(joints.get(0, 0))
                  * Proximal.len_pulley,
              Forearm.torque_spring);
    }
  }
}
