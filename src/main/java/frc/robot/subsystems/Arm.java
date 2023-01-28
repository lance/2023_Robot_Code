package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Motor controllers
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
//import com.revrobotics.CANSparkMaxLowLevel.ControlType;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax.IdleMode;

//Constants
import frc.robot.Constants.CanId;

public class Arm extends SubsystemBase {
  //Object initialization motor controllers
  private final CANSparkMax m_shoulderNeo1 = new CANSparkMax(CanId.shoulderNEO1, MotorType.kBrushless);
  private final CANSparkMax m_shoulderNeo2 = new CANSparkMax(CanId.shoulderNEO2, MotorType.kBrushless);
  private final CANSparkMax m_elbowNEO = new CANSparkMax(CanId.elbowNEO, MotorType.kBrushless);
  private final WPI_TalonSRX m_leftLead = new WPI_TalonSRX(CanId.turret);

  
  public Arm() {
    m_shoulderNeo2.follow(m_shoulderNeo1);
  }
  
  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    m_shoulderNeo1.setIdleMode(nMode);
    m_shoulderNeo2.setIdleMode(nMode);
  }

  //TODO figure out how to set with current - Current PIDS
  public void setTurretCurrent(){}
  public void setShoulderCurrent(){}
  public void setElbowCurrent(){}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}