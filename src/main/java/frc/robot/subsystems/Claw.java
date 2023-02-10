// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motionMagicLibrary;
import frc.robot.Constants.MotionMagicConstants;

public class Claw extends SubsystemBase {
  public TalonFX clawMotor = new TalonFX(11);
  public double m_targetPos;

  public Claw() {
    clawMotor.configFactoryDefault();
    motionMagicLibrary.setMotionMagicMotorParameters(clawMotor);
    // clawMotor.setInverted(true);
  }

  public void setClawPosition(double position) {
    m_targetPos = position;
    clawMotor.set(ControlMode.MotionMagic, m_targetPos);
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
