// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.motionMagicLibrary;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  WPI_TalonSRX elevatorMotor1 = new WPI_TalonSRX(14);
  VictorSPX elevatorMotor2 = new VictorSPX(13);
  double targetPos = 0;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    motionMagicLibrary.setMotionMagicMotorParameters(elevatorMotor1);
    elevatorMotor1.setInverted(false);
    elevatorMotor2.setInverted(true);
  }

  public void motionMagicElevator(double position) {
    targetPos = position;
    elevatorMotor1.set(ControlMode.MotionMagic, targetPos);
    elevatorMotor2.follow(elevatorMotor1);
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
