package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.armState;
import frc.robot.subsystems.Arm;
import java.util.HashMap;

public class ArmDefaultTrajectories {
  public HashMap<String, ArmTrajectory> trajectories = new HashMap<>();
  private Arm arm;

  public ArmDefaultTrajectories(Arm arm) {
    this.arm = arm;

    trajectories.put("INIT_HOME", new ArmTrajectory(simpleProfile(.07, 0.08, 0.18, 0.16)));
    trajectories.put("HOME_INIT", trajectories.get("INIT_HOME").reverse());

    var start = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.18, .16, 0, 0);
    var mid = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.45, .16, -0.75, 0.25);
    var end = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.65, -.08, 0, 0);
    trajectories.put(
        "HOME_GROUND",
        new ArmTrajectory(complexProfile(start, mid))
            .concatenate(new ArmTrajectory(complexProfile(mid, end))));

    trajectories.put("GROUND_HOME", trajectories.get("HOME_GROUND").reverse());

    start = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(1.16, 1.20, 0, 0);
    mid = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.65, .9, 0.1, -0.5);
    end = new MatBuilder<>(Nat.N4(), Nat.N1()).fill(.18, .16, 0, 0);
    trajectories.put(
        "L3_HOME",
        new ArmTrajectory(complexProfile(start, mid))
            .concatenate(new ArmTrajectory(complexProfile(mid, end))));
    trajectories.put("HOME_L3", trajectories.get("L3_HOME").reverse());

    trajectories.put("L2_HOME", new ArmTrajectory(simpleProfile(.9, .86, .18, .16)));
    trajectories.put("HOME_L2", trajectories.get("L2_HOME").reverse());

    trajectories.put("HOME_DOUBLESUB", new ArmTrajectory(simpleProfile(.18, 0.16, 0.65, 0.87)));
    trajectories.put("DOUBLESUB_HOME", trajectories.get("HOME_DOUBLESUB").reverse());

    // Neutral trajectories

    trajectories.put("HOME_NEUTRAL", new ArmTrajectory(simpleProfile(.17, .16, .49, .49)));
    trajectories.put("NEUTRAL_HOME", new ArmTrajectory(simpleProfile(.49, 0.49, 0.18, 0.16)));

    trajectories.put("L3_NEUTRAL", new ArmTrajectory(simpleProfile(1.16, 1.20, .49, .49)));
    trajectories.put("NEUTRAL_L3", trajectories.get("L3_NEUTRAL").reverse());

    trajectories.put("NEUTRAL_L2", new ArmTrajectory(simpleProfile(.49, .49, .9, .86)));
    trajectories.put("L2_NEUTRAL", new ArmTrajectory(simpleProfile(.9, .86, .49, .49)));

    trajectories.put("NEUTRAL_GROUND", new ArmTrajectory(simpleProfile(.49, .49, .65, -.08)));
    trajectories.put("GROUND_NEUTRAL", new ArmTrajectory(simpleProfile(.65, -.08, .49, .49)));

    trajectories.put("NEUTRAL_DOUBLESUB", new ArmTrajectory(simpleProfile(.49, .49, .65, 0.87)));
    trajectories.put("DOUBLESUB_NEUTRAL", new ArmTrajectory(simpleProfile(.87, .9, .49, .49)));
  }

  public Pair<TrapezoidProfile, TrapezoidProfile> simpleProfile(
      double startx, double starty, double endx, double endy) {
    return arm.motionProfile(
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(startx, starty),
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(endx, endy));
  }

  public Pair<TrapezoidProfile, TrapezoidProfile> complexProfile(
      Matrix<N4, N1> start, Matrix<N4, N1> end) {
    return arm.motionProfileVelocity(start, end);
  }

  public ArmTrajectory getTrajectory(Pair<armState, armState> trajPair) {
    return trajectories.get((trajPair.getFirst().name()) + "_" + trajPair.getSecond().name());
  }
}
