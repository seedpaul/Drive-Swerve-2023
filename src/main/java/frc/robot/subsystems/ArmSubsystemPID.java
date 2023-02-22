// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystemPID extends SubsystemBase {

  private static ArmSubsystemPID m_inst = null;

  private CANSparkMax m_motor;
  private SparkMaxPIDController m_pidController;
  private SparkMaxAbsoluteEncoder m_AbsoluteEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput,targetPosition;

  public static ArmSubsystemPID getInstance() {
    if (m_inst == null) {
      m_inst = new ArmSubsystemPID();
    }
    return m_inst;
  }

  /** Creates a new Arm. */
  private ArmSubsystemPID() {

    m_motor = new CANSparkMax(Constants.ArmSubsystemConstants.SparkMaxDeviceID, MotorType.kBrushless);
    m_motor.restoreFactoryDefaults();

    m_AbsoluteEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    m_AbsoluteEncoder.setPositionConversionFactor(360);
    m_AbsoluteEncoder.setVelocityConversionFactor(1);
    m_AbsoluteEncoder.setInverted(false);
    m_AbsoluteEncoder.setZeroOffset(11.5735374);
  
    m_pidController = m_motor.getPIDController();
    m_pidController.setFeedbackDevice(m_AbsoluteEncoder);
    // m_pidController.setPositionPIDWrappingEnabled(true);
    // m_pidController.setPositionPIDWrappingMaxInput(kMaxOutput);
    // m_pidController.setPositionPIDWrappingMinInput(kMaxOutput);

    m_pidController.setP(0.125);
    m_pidController.setI(0);
    m_pidController.setD(1);
    m_pidController.setFF(0);
    m_pidController.setOutputRange(-1, 1);

    m_motor.setIdleMode(IdleMode.kBrake);
    m_motor.setSmartCurrentLimit(80);
    m_motor.setClosedLoopRampRate(.5);
    // m_motor.setSoftLimit(SoftLimitDirection.kForward, 75);
    //m_motor.setSoftLimit(SoftLimitDirection.kReverse, 0);
    // m_motor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_motor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    // Save the SPARK MAX configurations. If a SPARK MAX browns out during
    // operation, it will maintain the above configurations.
    m_motor.burnFlash();

    this.targetPosition = 0;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Encoder", m_AbsoluteEncoder.getPosition());
    SmartDashboard.putNumber("Arm target position", targetPosition);
  }

  public void up() {
    m_pidController.setReference(1, CANSparkMax.ControlType.kDutyCycle);
  }

  public void down() {
    m_pidController.setReference(-1, CANSparkMax.ControlType.kDutyCycle);
  }
  
  public void stop() {
    m_pidController.setReference(0.0, CANSparkMax.ControlType.kDutyCycle);
  }

  private void setPosition(double position){
    m_pidController.setReference(position, CANSparkMax.ControlType.kPosition,0,0, ArbFFUnits.kPercentOut);
  }

  public void top(){
    this.targetPosition = Constants.ArmSubsystemConstants.ArmPositions.top;
    this.setPosition(Constants.ArmSubsystemConstants.ArmPositions.top);
  }

  public void middle(){
    this.targetPosition = Constants.ArmSubsystemConstants.ArmPositions.middle;
    this.setPosition(Constants.ArmSubsystemConstants.ArmPositions.middle);
  }

  public void floor(){
    this.targetPosition = Constants.ArmSubsystemConstants.ArmPositions.floor;
    this.setPosition(Constants.ArmSubsystemConstants.ArmPositions.floor);
  }
}
