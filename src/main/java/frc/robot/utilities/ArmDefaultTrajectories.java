package frc.robot.utilities;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.Arm;
import java.util.HashMap;

public class ArmDefaultTrajectories {
  private HashMap<String, ArmTrajectory> trajectories = new HashMap<>();
  private Arm arm;

  public ArmDefaultTrajectories(Arm arm) {
    this.arm = arm;

    trajectories.put("init_to_home", new ArmTrajectory(simpleProfile(.07, 0.08, 0.14, 0.12)));
    trajectories.put("home_to_init", new ArmTrajectory(simpleProfile(.14, 0.12, 0.07, 0.08)));

    trajectories.put("home_to_ground", new ArmTrajectory(simpleProfile(.14, .12, .65, -.04)));
    trajectories.put("ground_to_home", new ArmTrajectory(simpleProfile(.65, -.04, .14, .12)));

    trajectories.put("home_to_L3", new ArmTrajectory(simpleProfile(.14, .12, 1.3, 1.03)));
    trajectories.put("L3_to_home", new ArmTrajectory(simpleProfile(1.3, 1.03, .14, .12)));

    trajectories.put("home_to_L2", new ArmTrajectory(simpleProfile(.14, .12, .9, .49)));
    trajectories.put("L2_to_home", new ArmTrajectory(simpleProfile(.9, .49, .14, .12)));

    // trajectories.put("home_to_doublesub", new ArmTrajectory(simpleProfile(.14, 0.12, 0, 0)))
  }

  public Pair<TrapezoidProfile, TrapezoidProfile> simpleProfile(
      double startx, double starty, double endx, double endy) {
    return arm.motionProfile(
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(startx, starty),
        new MatBuilder<N2, N1>(Nat.N2(), Nat.N1()).fill(endx, endy));
  }

  public ArmTrajectory getTrajectory(String name) {
    return trajectories.get(name);
  }
}
