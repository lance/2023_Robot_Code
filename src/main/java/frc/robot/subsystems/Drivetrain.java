

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;


import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

//Kinematics and drivetrain abstractions
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

//Math
import java.lang.Math;
import edu.wpi.first.math.MathUtil;

import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  //Initalize motor controllers
  private final WPI_TalonSRX m_leftLead = new WPI_TalonSRX(Constants.CanId.leftDriveLead);
  private final WPI_TalonSRX m_leftFollow = new WPI_TalonSRX(Constants.CanId.leftDriveFollow);
  private final WPI_TalonSRX m_rightLead = new WPI_TalonSRX(Constants.CanId.rightDriveLead);
  private final WPI_TalonSRX m_rightFollow = new WPI_TalonSRX(Constants.CanId.rightDriveFollow);

  //Create motor controller groups
  private final MotorControllerGroup m_left = new MotorControllerGroup(m_leftLead, m_leftFollow); 
  private final MotorControllerGroup m_right = new MotorControllerGroup(m_rightLead, m_rightFollow);

  //Create Drivetrain controllers and kinematics objects
  private SimpleMotorFeedforward m_lFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Left.kS, Constants.Drive.Feedforward.Left.kV, Constants.Drive.Feedforward.Left.kA);
  private SimpleMotorFeedforward m_rFeedforward = new SimpleMotorFeedforward(Constants.Drive.Feedforward.Right.kS, Constants.Drive.Feedforward.Right.kV, Constants.Drive.Feedforward.Right.kA);
  private DifferentialDriveKinematics m_driveKinematics = new DifferentialDriveKinematics(Constants.Drive.kTrackWidth);


  //Constructor taking no arguments, all relevant values are defined in Constants.java
  public Drivetrain() {
    //Set one drivetrain pair to run reverse so both drive forward on the positive direction (Which group selected by Constants.Drive.kInvertDrive; left = False)
    m_left.setInverted(!Constants.Drive.kInvertDrive);
    m_right.setInverted(Constants.Drive.kInvertDrive);
  }

  //Enable or disable brake mode on the motors
  public void brakeMode(boolean mode){
    NeutralMode nMode = NeutralMode.Coast;
    if (mode) nMode = NeutralMode.Brake;

    m_leftLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
    m_rightLead.setNeutralMode(nMode);
    m_leftFollow.setNeutralMode(nMode);
  }

  //Simple arcade drive that uses a percentage (-1.00 to 1.00) of the max forward and angular speeds to drive the chassis at
  public void arcadeDrive(double linearPercent, double angularPercent){
    linearPercent = MathUtil.clamp(linearPercent, -1, 1);
    angularPercent = MathUtil.clamp(angularPercent, -1, 1);

    double maxAngularSpeed = m_driveKinematics.toChassisSpeeds(new DifferentialDriveWheelSpeeds(Constants.Drive.Rate.maxSpeed, Constants.Drive.Rate.maxSpeed)).omegaRadiansPerSecond;
    driveChassisSpeeds(new ChassisSpeeds(Constants.Drive.Rate.maxSpeed * linearPercent, 0, maxAngularSpeed * angularPercent));
  }

  //Simple tank drive that uses a percentage (-1.00 to 1.00) of the max left and right speeds to drive the wheels at
  public void tankDrive(double leftPercent, double rightPercent){
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1 , 1);

    driveWheelSpeeds(new DifferentialDriveWheelSpeeds(Constants.Drive.Rate.maxSpeed * leftPercent, Constants.Drive.Rate.maxSpeed * rightPercent));
  }

  //Set the appropriate motor voltages for a desired set of wheel speeds
  public void driveWheelSpeeds(DifferentialDriveWheelSpeeds wheelSpeeds){
    m_left.setVoltage(m_lFeedforward.calculate(wheelSpeeds.leftMetersPerSecond));
    m_right.setVoltage(m_rFeedforward.calculate(wheelSpeeds.rightMetersPerSecond));
  }

  //Set the appropriate motor voltages for a desired set of linear and angular chassis speeds
  public void driveChassisSpeeds(ChassisSpeeds chassisSpeeds){
    driveWheelSpeeds(m_driveKinematics.toWheelSpeeds(chassisSpeeds));
  }

  //Drive the motors at a given voltage
  public void driveVoltages(double leftVoltage, double rightVoltage){
    m_left.setVoltage(leftVoltage);
    m_right.setVoltage(rightVoltage);
  }

  //Utility function to map joystick input nonlinearly for driver "feel"
  public static double NonLinear(double input){ return Math.copySign(input * input, input);}
}