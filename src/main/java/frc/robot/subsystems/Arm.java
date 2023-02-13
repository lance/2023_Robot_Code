package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax shoulderNeo1 =
      new CANSparkMax(CanId.shoulderNeo1, MotorType.kBrushless);
  private final CANSparkMax shoulderNeo2 =
      new CANSparkMax(CanId.shoulderNeo2, MotorType.kBrushless);
  private final CANSparkMax elbowNeo = new CANSparkMax(CanId.elbowNeo, MotorType.kBrushless);
  private final WPI_TalonSRX leftLead = new WPI_TalonSRX(CanId.turret);
  private final double As = 0.0;
  private final double Ae = 0.0;

  public Arm() {
    shoulderNeo2.follow(shoulderNeo1);
  }

  // Enable or disable brake mode on the motors
  public void brakeMode(boolean mode) {
    IdleMode nMode = IdleMode.kCoast;
    if (mode) nMode = IdleMode.kBrake;

    shoulderNeo1.setIdleMode(nMode);
    shoulderNeo2.setIdleMode(nMode);
  }

  // TODO figure out how to set with current - Current PIDS
  public void setTurretCurrent() {}

  public void setShoulderCurrent() {}

  public void setElbowCurrent() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }


    public void kinematics(double Lp, double Lf) {
         
         double xG = Lp*Math.cos(As)+Lf*Math.cos(Ae);
         double yG = Lp*Math.sin(As)+Lf*Math.sin(Ae);
    }
}
}
