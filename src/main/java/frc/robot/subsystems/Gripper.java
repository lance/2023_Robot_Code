package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.kGripper;
import frc.robot.utilities.PicoColorSensor;
import frc.robot.utilities.PicoColorSensor.RawColor;

public class Gripper extends SubsystemBase {
  GamePiece gameState;
  // Initialize Motorcontroller objects
  private final CANSparkMax gripperNEO1 = new CANSparkMax(CanId.gripperNEO1, MotorType.kBrushless);
  private final CANSparkMax gripperNEO2 = new CANSparkMax(CanId.gripperNEO2, MotorType.kBrushless);
  // Initialize MotorControllerGroup for gripper
  private final MotorControllerGroup GripperControllerGroup =
      new MotorControllerGroup(gripperNEO1, gripperNEO2);

  private PicoColorSensor colorSensor;

  public Gripper() {
    // Sets the motor controllers to inverted if needed
    gripperNEO1.setInverted(!kGripper.inverted);
    gripperNEO2.setInverted(kGripper.inverted);
    gripperNEO1.setSmartCurrentLimit(kGripper.stallCurrentLimit, kGripper.freeCurrentLimit);

    colorSensor = new PicoColorSensor();
    colorSensor.setDebugPrints(false);

    this.setDefaultCommand(holdCommand(getGamePiece()));
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
              gameState = getGamePiece();
              setVoltage(gameState==GamePiece.CONE?kGripper.holdingVoltageCone:kGripper.holdingVoltageKube);
            })
        .until(() -> getGamePiece() != GamePiece.NONE);
  }

  public Command ejectCommand() {
    return this.startEnd(
            () -> setVoltage(kGripper.ejectVel),
            () -> {
              setVoltage(0);
              gameState = getGamePiece();
            })
        .until(() -> getGamePiece() == GamePiece.NONE);
  }

  public Command holdCommand(GamePiece Piece) {
    return this.startEnd(
        () -> {
          if (Piece == GamePiece.CONE) {
            setVoltage(kGripper.holdingVoltageCone);
          } else if (Piece == GamePiece.KUBE) {
            setVoltage(kGripper.holdingVoltageKube);
          }
        },
        () -> {
          setVoltage(0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
