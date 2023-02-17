package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CanId;
import frc.robot.Constants.kArm.Dimensions;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;

public class Arm extends SubsystemBase {
  // Object initialization motor controllers
  private final CANSparkMax shoulderNeo1 =
      new CANSparkMax(CanId.shoulderNeo1, MotorType.kBrushless);
  private final CANSparkMax shoulderNeo2 =
      new CANSparkMax(CanId.shoulderNeo2, MotorType.kBrushless);
  private final CANSparkMax elbowNeo = new CANSparkMax(CanId.elbowNeo, MotorType.kBrushless);
  private final WPI_TalonSRX leftLead = new WPI_TalonSRX(CanId.turret);

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

  public void kinematics(double theta_s, double theta_e) {
    double xG = Dimensions.Lp * Math.cos(theta_s) + Dimensions.Lf * Math.cos(theta_e);
    double yG = Dimensions.Lp * Math.sin(theta_s) + Dimensions.Lf * Math.sin(theta_e);
  }

  public Matrix inverseKinematics(double xG,double yG){
    double r = Math.sqrt(Math.pow(xG, 2) + Math.pow(yG, 2));
    double theta_s = Math.atan(yG/xG) + Math.acos((Math.pow(r, 2) + Math.pow(Dimensions.Lp, 2) - Math.pow(Dimensions.Lf,2))/(2*r*Dimensions.Lp));
    double theta_e = Math.atan(yG/xG) - Math.acos((Math.pow(r, 2) + Math.pow(Dimensions.Lf, 2) - Math.pow(Dimensions.Lp,2))/(2*r*Dimensions.Lf));
    return new MatBuilder<>(Nat.N2(), Nat.N1()).fill(theta_s, theta_e);
  }

}
