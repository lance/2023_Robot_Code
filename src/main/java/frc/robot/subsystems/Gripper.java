package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.kGripper;
import frc.robot.utilities.PicoColorSensor;
import frc.robot.utilities.PicoColorSensor.RawColor;

public class Gripper extends SubsystemBase {
  private GamePiece gripperState;
  // Initialize Motorcontroller objects
  private final CANSparkMax gripperNEO1 = new CANSparkMax(CanId.gripperNEO1, MotorType.kBrushless);
  private final CANSparkMax gripperNEO2 = new CANSparkMax(CanId.gripperNEO2, MotorType.kBrushless);
  // Initialize MotorControllerGroup for gripper
  private final MotorControllerGroup GripperControllerGroup =
      new MotorControllerGroup(gripperNEO1, gripperNEO2);

  private PicoColorSensor colorSensor;

  private DataLog log = DataLogManager.getLog();
  private StringLogEntry logGripperState = new StringLogEntry(log, "Gripper/gripperState");
  private DoubleLogEntry logProximity = new DoubleLogEntry(log, "Gripper/proximity");
  private DoubleArrayLogEntry logColorChannels =
      new DoubleArrayLogEntry(log, "Gripper/colorChannels");

  public Gripper() {
    // Sets the motor controllers to inverted if needed
    gripperNEO1.setInverted(!kGripper.inverted);
    gripperNEO2.setInverted(kGripper.inverted);
    gripperNEO1.setSmartCurrentLimit(kGripper.stallCurrentLimit, kGripper.freeCurrentLimit);

    colorSensor = new PicoColorSensor();
    colorSensor.setDebugPrints(false);

    this.setDefaultCommand(holdCommand());
  }

  // Sets the voltage of motors
  public void setVoltage(double voltage) {
    GripperControllerGroup.setVoltage(voltage);
  }

  public RawColor getRawColor() {
    return colorSensor.getRawColor0();
  }

  public int getProximity() {
    return colorSensor.getProximity0();
  }

  public boolean getConnected() {
    return colorSensor.isSensor0Connected();
  }

  public GamePiece getGamePiece() {
    int proximity = colorSensor.getProximity0();
    RawColor color = colorSensor.getRawColor0();

    if (proximity > kGripper.proximityThreshold) {
      double colorRatio = (double) color.blue / (double) color.green;
      if (4 < colorRatio && colorRatio < 8) return GamePiece.CONE;
      else if (0 < colorRatio && colorRatio < 2) return GamePiece.KUBE;
    }
    return GamePiece.NONE;
  }

  public Command intakeCommand(GamePiece Piece) {
    return this.startEnd(
            () -> {
              if (Piece == GamePiece.CONE) {
                setVoltage(kGripper.intakeVoltageCone);
              } else if (Piece == GamePiece.KUBE) {
                setVoltage(kGripper.intakeVoltageKube);
              }
            },
            () -> {
              gripperState = getGamePiece();
              setVoltage(
                  gripperState == GamePiece.CONE
                      ? kGripper.holdingVoltageCone
                      : kGripper.holdingVoltageKube);
            })
        .until(() -> getGamePiece() != GamePiece.NONE);
  }

  public Command ejectCommand() {
    return this.startEnd(
            () -> setVoltage(kGripper.ejectVoltage),
            () -> {
              setVoltage(0);
              gripperState = GamePiece.NONE;
            })
        .until(() -> getGamePiece() == GamePiece.NONE);
  }

  public Command holdCommand() {
    return this.runOnce(
        () -> {
          if (gripperState == GamePiece.CONE) {
            setVoltage(kGripper.holdingVoltageCone);
          } else if (gripperState == GamePiece.KUBE) {
            setVoltage(kGripper.holdingVoltageKube);
          } else setVoltage(0);
        });
  }

  @Override
  public void periodic() {
    var color = getRawColor();
    logGripperState.append(String.valueOf(gripperState));
    logProximity.append(getProximity());
    logColorChannels.append(new double[] {color.red, color.green, color.blue});
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
