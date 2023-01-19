package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import frc.robot.Constants.CanId;

public class Arm extends SubsystemBase {
  //Object initialization motor controllers
  private final CANSparkMax m_shoulderNeo1 = new CANSparkMax(CanId.shoulderNEO1, MotorType.kBrushless);
  private final CANSparkMax m_shoulderNEO2 = new CANSparkMax(CanId.shoulderNEO2, MotorType.kBrushless);
  private final CANSparkMax m_elbowNEO = new CANSparkMax(CanId.elbowNEO, MotorType.kBrushless);
  private final WPI_TalonSRX m_leftLead = new WPI_TalonSRX(CanId.turret);
  private final MotorControllerGroup m_shoulderMotorControllerGroup = new MotorControllerGroup(m_shoulderNeo1, m_shoulderNEO2);





  public Arm() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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