package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;

//Motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

//Constants
import frc.robot.Constants.CanId;
import frc.robot.Constants.ArmConstants.GripperConstants;

public class Gripper extends SubsystemBase {
  //Initialize Motorcontroller objects
  private final CANSparkMax m_gripperNeo1 = new CANSparkMax(CanId.gripperNeo1, MotorType.kBrushless);
  private final CANSparkMax m_gripperNeo2 = new CANSparkMax(CanId.gripperNeo2, MotorType.kBrushless);
  //Initialize MotorControllerGroup for gripper
  private final MotorControllerGroup GripperControllerGroup = new MotorControllerGroup(m_gripperNeo1, m_gripperNeo2); 

  public Gripper() {
    //Sets the motor controllers to inverted if needed
    m_gripperNeo1.setInverted(!GripperConstants.inverted);
    m_gripperNeo2.setInverted(GripperConstants.inverted);
  }

  //Sets the voltage of motors
  public void setVoltage(double voltage){
    GripperControllerGroup.setVoltage(voltage);
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
