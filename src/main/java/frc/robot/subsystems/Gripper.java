package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.GripperConstants;
import frc.robot.Constants.CanId;
import frc.robot.Constants.GamePiece;
import frc.robot.Constants.kSensors;
import frc.robot.utilities.PicoColorSensor;
import frc.robot.utilities.PicoColorSensor.RawColor;

public class Gripper extends SubsystemBase {
  // Initialize Motorcontroller objects
  private final CANSparkMax gripperNeo1 = new CANSparkMax(CanId.gripperNeo1, MotorType.kBrushless);
  private final CANSparkMax gripperNeo2 = new CANSparkMax(CanId.gripperNeo2, MotorType.kBrushless);
  // Initialize MotorControllerGroup for gripper
  private final MotorControllerGroup GripperControllerGroup =
      new MotorControllerGroup(gripperNeo1, gripperNeo2);

  private PicoColorSensor colorSensor;

  public Gripper() {
    // Sets the motor controllers to inverted if needed
    gripperNeo1.setInverted(!GripperConstants.inverted);
    gripperNeo2.setInverted(GripperConstants.inverted);

    colorSensor = new PicoColorSensor();
    colorSensor.setDebugPrints(false);
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

    if (proximity > kSensors.proximityThreshold) {
      double colorRatio = (double) color.blue / (double) color.green;
      if (4 < colorRatio && colorRatio < 8) return GamePiece.CONE;
      else if (0 < colorRatio && colorRatio < 2) return GamePiece.KUBE;
    }
    return GamePiece.NONE;
  }

  public Command intakeCommand() {
    return this.startEnd(() -> setVoltage(3), () -> setVoltage(0))
        .until(() -> getGamePiece() != GamePiece.NONE);
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
