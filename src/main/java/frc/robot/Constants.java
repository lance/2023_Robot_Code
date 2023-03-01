package frc.robot;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class OperatorInterface {
    public static final int primaryController = 0;
  }

  public static final class kGripper {
    public static final boolean inverted = true;
    public static final int proximityThreshold = 100;
    // TODO Tune these constants:
    public static final int intakeVoltageCone = 4;
    public static final int intakeVoltageKube = 3;
    public static final int ejectVel = -3;
    public static final int holdingVoltageCone = 3;
    public static final int holdingVoltageKube = 4;
    public static final int stallCurrentLimit = 5;
    public static final int freeCurrentLimit = 7;
  }

  public static final class kArm {

    public static final class Constraints {
      public static final double Velocity = 5;
      public static final double Acceleration = 1;
    }

    public static final class Motors {
      public static final double stall_torque = 0.0;
      public static final double stall_current = 0.0;
      public static final double free_speed = 0.0;
    }

    public static final class Encoders {
      public static final int PPR = 1024;

      public static final class Proximal {
        public static final double initial = 0;
        public static final double gear_ratio = 1;
        public static final int absPort = 0;
        public static final int APort = 1;
        public static final int BPort = 2;
      }

      public static final class Forearm {
        public static final double initial = 0;
        public static final double gear_ratio = 1;
        public static final int absPort = 3;
        public static final int APort = 4;
        public static final int BPort = 5;
      }

      public static final class Turret {
        public static final double initial = 0;
        public static final double gear_ratio = 1;
        public static final int absPort = 6;
        public static final int APort = 7;
        public static final int BPort = 8;
      }
    }

    // kg, m, kg*m^2, rad, N/m
    public static final class Proximal {
      public static final double mass = 0.0;
      public static final double length = 0.0;
      public static final double inertia = 0.0;
      public static final double radius = 0.0;
      public static final double gear_ratio = 0.0;
      public static final double num_motors = 2;
      public static final double len_pulley = 0.0;
      public static final double len_anchor = 0.0;
      public static final double angle_pulley = Units.degreesToRadians(15);
      public static final double angle_anchor = Units.degreesToRadians(115);
      public static final double k_spring = 0.0;
    }

    // kg, m, kg*m^2, rad, Nm
    public static final class Forearm {
      public static final double mass = 0.0;
      public static final double length = 0.0;
      public static final double inertia = 0.0;
      public static final double radius = 0.0;
      public static final double gear_ratio = 0.0;
      public static final double num_motors = 1;
      public static final double torque_spring = 14.915;
    }

    public static final class Turret {
      public static final double ks = 0;
      public static final double kv = 0;
      public static final double ka = 0;
      public static final double kp = 0;
      public static final double ki = 0;
      public static final double kd = 0;
    }

    public static final class Feedback {
      public static final double proximal_kP = 0;
      public static final double proximal_kD = 0;
      public static final double forearm_kP = 0;
      public static final double forearm_kD = 0;
    }
  }

  public static final class CanId {
    public static final int leftDriveLead = 2;
    public static final int leftDriveFollow = 3;
    public static final int rightDriveLead = 4;
    public static final int rightDriveFollow = 5;
    public static final int proximalNEO1 = 20;
    public static final int proximalNEO2 = 21;
    public static final int forearmNEO = 22;
    public static final int turret = 23;
    public static final int gripperNEO1 = 24;
    public static final int gripperNEO2 = 25;
  }

  public static final class kDrivetrain {
    public static final class Feedforward {
      // Feedforwards from sysid
      public final class Linear {
        public static final double kS = 0.093288;
        public static final double kV = 2.507;
        public static final double kA = 0.23549;
      }

      public final class Angular {
        public static final double kS = 0.10491;
        public static final double kV = 2.5962;
        public static final double kA = 0.13804;
      }
    }

    public final class TrajectoryConstants {
      public static final int kMaxSpeedMetersPerSecond = 12;
      public static final int kMaxAccelerationMetersPerSecondSquared = 12;
      public static final boolean setReversed = true;
    }

    public static final class Dimensions {
      public static final double wheelCircumferenceMeters = Units.inchesToMeters(6 * Math.PI);
      public static final double trackWidthMeters = 0.58208;
      public static final boolean kInvertDrive = false;
    }

    public static final class Rate {
      // Speeds in m/s rotations in rad/s
      public static final double maxSpeed = 4.7;
      public static final double maxAccel = 0.9 * 9.81;
      public static final double maxAngularSpeed = 8;
      public static final double maxAngularAccel = 15;
      public static final double driverSpeed = 3.5;
      public static final double driverAngularSpeed = 4;
      public static final double driverAccel = 4;
    }

    public static final class PathFollowing {
      public static final Vector<N5> qelems =
          VecBuilder.fill(.075, .075, Units.degreesToRadians(3), .1, .1);
      public static final Vector<N2> relems = VecBuilder.fill(12.0, 12.0);
    }

    public final class Encoders {
      public static final int rightAPort = 0;
      public static final int rightBPort = 1;
      public static final int leftAPort = 2;
      public static final int leftBPort = 3;

      public static final double PPR = 1024;
      public static final double gearing = 34.0 / 18.0;
    }
  }

  public static final class kVision {
    // Cam mounted facing forward, half a meter behind center, half a meter up from center.
    public static final Transform3d aprilTagCameraPositionTransform =
        new Transform3d(new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
  }

  public static final class kIndications {
    public static final int ledPort = 0;
    public static final int ledLength = 300;
  }

  public enum GamePiece {
    CONE,
    KUBE,
    NONE
  }
}
