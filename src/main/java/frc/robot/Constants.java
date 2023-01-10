

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final int primaryController = 0;

    public final class CanId{
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
    }

    public final class Drive{
        public final class Feedforward{
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
        public static final double kTrackWidth = 1; // m/s
        public static final boolean kInvertDrive = false;

        public final class Rate{
            //Speeds in m/s rotations in rad/s
            public static final double maxSpeed = 5.45;
            public static final double driverSpeed = 4;
            public static final double driverAngularSpeed = 3;
            public static final double driverAccel = 5;
            public static final double driverDeccel = 10;
        }
    }

    public static final boolean[] motorConfigs = {true,false,false};
}
