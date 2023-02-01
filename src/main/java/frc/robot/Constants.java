

package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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
  public static final class OperatorInterface{
    public static final int primaryController = 0;
  }
  public static final class ArmConstants {

    public static final class GripperConstants {
      public static final boolean inverted = true;
    }
  }
  public static final class CanId{
    public static final int climberLiftLead = 1;
    public static final int climberLiftFollow = 2;
    public static final int climberRotate = 3;
    public static final int intakeLower = 5;
    public static final int upperConveyor = 4;
    public static final int shooterLead = 6;
    public static final int shooterFollow = 7;
    public static final int leftDriveLead = 13;
    public static final int leftDriveFollow = 12;
    public static final int rightDriveLead = 11;
    public static final int rightDriveFollow = 10;
    public static final int compressor = 8;
    public static final int solenoidPort = 0;
    public static final int shoulderNeo1 = 20;
    public static final int shoulderNeo2 = 21;
    public static final int elbowNeo = 22;
    public static final int turret = 23;
    public static final int gripperNeo1 = 24;
    public static final int gripperNeo2 = 25;
  }

  public static final class Drivetrain{
    public static final class Feedforward{
      //Feedforwards from sysid
      public final class Left{
        public static final double kS = 0.56131;
        public static final double kV = 2.065;
        public static final double kA = 0.37539;
      }
      public final class Right{
        public static final double kS = 0.55809;
        public static final double kV = 2.0644;
        public static final double kA = 0.19512;
      }
      
    }
    public final class TrajectoryConstants{
      public static final int kMaxSpeedMetersPerSecond = 12;
      public static final int kMaxAccelerationMetersPerSecondSquared = 12;
      public static final boolean setReversed = true;
    }
    public static final class Dimensions{
      public static final double wheelCircumferenceMeters = Units.inchesToMeters(6*Math.PI);
      //TODO Measure Trackwidth
      public static final double trackWidthMeters = Units.inchesToMeters(30);
      public static final boolean kInvertDrive = true;
      }

    public static final class Rate{
      //Speeds in m/s rotations in rad/s
      public static final double maxSpeed = 5.45;
      public static final double driverSpeed = 4;
      public static final double driverAngularSpeed = 3;
      public static final double driverAccel = 5;
      public static final double driverDeccel = 10;
    }
    public final class PIDs{
      public final class Left{
        public static final double kS = 0.56131;
        public static final double kV = 2.065;
        public static final double kA = 0.37539;
      }
      public final class Right{
        public static final double kS = 0.55809;
        public static final double kV = 2.0644;
        public static final double kA = 0.19512;
      }
    }
    public final class Encoders{
      public static final int rightAPort = 2;
      public static final int rightBPort = 3;
      public static final int leftBPort = 0;
      public static final int leftAPort = 1;
    
      public static final int PPR = 248;
    }
  }

  public static final class Vision{
    public static final Transform3d aprilTagCameraPositionTransform = new Transform3d( //Cam mounted facing forward, half a meter behind center, half a meter up from center.
      new Translation3d(-0.5, 0.0, 0.5),
      new Rotation3d(0,0,0));
  }
  public static final class kSensors{
    public static final int ledPort = 0;
    public static final int ledLength = 300;
    //For Color Sensor
    public static final int proximityThreshold = 100;
  }
  public enum GamePiece{
    CONE,
    KUBE,
    NONE
  }
}
