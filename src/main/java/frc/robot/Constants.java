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
    public static final int secondaryController = 1;

    public static final class Bindings {
      public static final int groundIntake = 3;
      public static final int L2 = 4;
      public static final int L3 = 6;
      public static final int L1 = 11;
    }
  }

  public static final class kGripper {
    public static final boolean inverted = true;
    public static final int proximityThreshold = 150;
    public static final double intakeVoltageCone = 5;
    public static final double intakeVoltageKube = 3;
    public static final double ejectVoltage = -6;
    public static final double holdingVoltageCone = 1;
    public static final double holdingVoltageKube = 1;
    public static final int stallCurrentLimit = 5;
    public static final int freeCurrentLimit = 7;
  }

  public static final class kArm {

    public static final class Constraints {
      public static final double Velocity = 1.25;
      public static final double Acceleration = .5;
    }

    public static final class Motors {
      public static final double stall_torque = 3.28;
      public static final double stall_current = 181;
      public static final double free_speed = Units.rotationsPerMinuteToRadiansPerSecond(5820);
    }

    public static final class Encoders {
      public static final int PPR = 2048;
      public static final long startUpTime = 800;

      public static final class Proximal {
        public static final double initial = Math.PI / 2 + Units.degreesToRadians(16);
        public static final double offset = -1.515485;
        public static final double gear_ratio = 50.0 / 18.0 * 12.0 / 44.0;
        public static final int absPort = 9;
        public static final int APort = 7;
        public static final int BPort = 8;
      }

      public static final class Forearm {
        public static final double initial = Units.degreesToRadians(-172);
        public static final double offset = 3.254898;
        public static final double gear_ratio = 16.0 / 34.0;
        public static final int absPort = 12;
        public static final int APort = 10;
        public static final int BPort = 11;
      }

      public static final class Turret {
        public static final double initial = 0;
        public static final double offset = .068;
        public static final double gear_ratio = -15.0 / 160.0;
        public static final int absPort = 6;
        public static final int APort = 4;
        public static final int BPort = 5;
      }
    }

    // kg, m, kg*m^2, rad, N/m
    public static final class Proximal {
      public static final double mass = 5.859506;
      public static final double length = 0.889;
      public static final double inertia = 0.509046682;
      public static final double radius = 0.3504184;
      public static final double gear_ratio = 320.0;
      public static final double num_motors = 2;
      public static final double len_pulley = 0.1293622;
      public static final double len_anchor = 0.1788414;
      public static final double angle_pulley = Units.degreesToRadians(27.5);
      public static final double angle_anchor = Units.degreesToRadians(180 - 30);
      public static final double k_spring = 1838 * 2;
    }

    // kg, m, kg*m^2, rad, Nm
    public static final class Forearm {
      public static final double mass = 4.75;
      public static final double length = 0.8255;
      public static final double inertia = 0.866469442;
      public static final double radius = 0.6303518;
      public static final double gear_ratio = 140.0;
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
      public static final double proximal_kP = 10;
      public static final double proximal_kD = 1;
      public static final double forearm_kP = 12;
      public static final double forearm_kD = 1;
    }
  }

  public static final class CanId {
    public static final int leftDriveLead = 2;
    public static final int leftDriveFollow = 3;
    public static final int rightDriveLead = 4;
    public static final int rightDriveFollow = 5;
    public static final int proximalNEO1 = 6;
    public static final int proximalNEO2 = 7;
    public static final int forearmNEO = 8;
    public static final int turret = 1;
    public static final int gripperNEO1 = 9;
    public static final int gripperNEO2 = 10;
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
    // Cam mounted facing forward, .243 meters ahead of center, .193 meters left of center, .229
    // meters up from ground.
    public static final Transform3d aprilTagCameraPositionTransform =
        new Transform3d(new Translation3d(0.243, 0.193, 0.229), new Rotation3d(0, 0, 0));
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
