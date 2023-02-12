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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motionMagicLibrary;
import frc.robot.Constants.MotionMagicConstants;

public class ClawPneumatics extends SubsystemBase {
  public DoubleSolenoid clawCylinder = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 14, 15);

  public ClawPneumatics() {
    // clawMotor.setInverted(true);
  }

  public void setGrip(boolean close) {
    if (close) {
      clawCylinder.set(Value.kReverse);
    } else {
      clawCylinder.set(Value.kForward);
    }
  }

  public void openClaw() {
    clawCylinder.set(Value.kForward);
  }

  public void closeClaw() {
    clawCylinder.set(Value.kReverse);
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
