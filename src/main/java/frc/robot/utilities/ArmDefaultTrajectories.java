package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.Constants.armState;
import frc.robot.subsystems.Arm;
import java.util.HashMap;

public class ArmDefaultTrajectories {
  public HashMap<String, ArmTrajectory> trajectories = new HashMap<>();
  private Arm arm;

  public ArmDefaultTrajectories(Arm arm) {
    this.arm = arm;

    trajectories.put("INIT_HOME", new ArmTrajectory(simpleProfile(.07, 0.08, 0.14, 0.12)));
    trajectories.put("HOME_INIT", new ArmTrajectory(simpleProfile(.14, 0.12, 0.07, 0.08)));

    trajectories.put(
        "HOME_GROUND",
        new ArmTrajectory(simpleProfile(.14, .12, .45, .12))
            .concatenate(new ArmTrajectory(simpleProfile(0.45, .12, .65, -.08))));
    trajectories.put(
        "GROUND_HOME",
        new ArmTrajectory(simpleProfile(.65, -.08, 0.45, .12))
            .concatenate(new ArmTrajectory(simpleProfile(.45, .12, 0.14, .12))));

    trajectories.put(
        "HOME_L3",
        new ArmTrajectory(simpleProfile(.14, .12, .65, .9))
            .concatenate(new ArmTrajectory(simpleProfile(.65, .9, 1.16, 1.26))));
    trajectories.put(
        "L3_HOME",
        new ArmTrajectory(simpleProfile(1.16, 1.26, .65, .9))
            .concatenate(new ArmTrajectory(simpleProfile(.65, .9, .14, .12))));

    trajectories.put("HOME_L2", new ArmTrajectory(simpleProfile(.14, .12, .9, .9)));
    trajectories.put("L2_HOME", new ArmTrajectory(simpleProfile(.9, .9, .14, .12)));

    trajectories.put("HOME_DOUBLESUB", new ArmTrajectory(simpleProfile(.14, 0.12, 0.65, 0.85)));
    trajectories.put("DOUBLESUB_HOME", new ArmTrajectory(simpleProfile(.65, 0.85, 0.14, 0.12)));

    // Neutral trajectories

    trajectories.put("HOME_NEUTRAL", new ArmTrajectory(simpleProfile(.14, .12, .49, .49)));
    trajectories.put("NEUTRAL_HOME", new ArmTrajectory(simpleProfile(.49, 0.49, 0.14, 0.12)));

    trajectories.put("NEUTRAL_L3", new ArmTrajectory(simpleProfile(.49, .49, 1.16, 1.26)));
    trajectories.put("L3_NEUTRAL", new ArmTrajectory(simpleProfile(1.16, 1.26, .49, .49)));

    trajectories.put("NEUTRAL_L2", new ArmTrajectory(simpleProfile(.49, .49, .9, .9)));
    trajectories.put("L2_NEUTRAL", new ArmTrajectory(simpleProfile(.9, .9, .49, .49)));

    trajectories.put("NEUTRAL_GROUND", new ArmTrajectory(simpleProfile(.49, .49, .65, -.08)));
    trajectories.put("GROUND_NEUTRAL", new ArmTrajectory(simpleProfile(.65, -.08, .49, .49)));

    trajectories.put("NEUTRAL_DOUBLESUB", new ArmTrajectory(simpleProfile(.49, .49, .65, 0.85)));
    trajectories.put("DOUBLESUB_NEUTRAL", new ArmTrajectory(simpleProfile(.85, .9, .49, .49)));
  }

  public Pair<TrapezoidProfile, TrapezoidProfile> simpleProfile(
      double startx, double starty, double endx, double endy) {
    return arm.motionProfile(
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(startx, starty),
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(endx, endy));
  }

  public ArmTrajectory getTrajectory(Pair<armState, armState> trajPair) {
    return trajectories.get((trajPair.getFirst().name()) + "_" + trajPair.getSecond().name());
  }
}
