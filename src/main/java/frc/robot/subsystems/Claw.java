// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.MotionMagicLibrary;

public class Claw extends SubsystemBase {
  public TalonFX clawMotor = new TalonFX(11);
  // public DigitalInput testInput = new DigitalInput(4);
  public double m_targetPos;

  public Claw() {
    clawMotor.configFactoryDefault();
    MotionMagicLibrary.setMotionMagicMotorParameters(clawMotor, 0.3, 0.01, 0.0, 0.01, 30000, 30000);
    clawMotor.setNeutralMode(NeutralMode.Brake);
    // clawMotor.setInverted(true);
  }

  public void zeroPosition() {
    clawMotor.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  public void setClawPosition(double position) {
    m_targetPos = position;
    clawMotor.set(ControlMode.MotionMagic, m_targetPos);
  }

  public void runMotorForward() {
    clawMotor.set(ControlMode.PercentOutput, -0.5);
  }

  public void runMotorBackward() {
    clawMotor.set(ControlMode.PercentOutput, 0.5);
  }

  public void stopMotor() {
    clawMotor.set(ControlMode.PercentOutput, 0.0);
  }

  public void changeMode(NeutralMode mode){
    clawMotor.setNeutralMode(mode);
  }

  public double getVelocity() {
    return clawMotor.getSelectedSensorVelocity();
  }

  public double getCurrentPosition() {
    return clawMotor.getSelectedSensorPosition();
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
