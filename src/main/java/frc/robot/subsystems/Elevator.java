// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static frc.robot.Constants.MotionMagicConstants.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.MotionMagicLibrary;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  WPI_TalonSRX elevatorMotor1 = new WPI_TalonSRX(9);
  VictorSPX elevatorMotor2 = new VictorSPX(10);
  public double targetPos = 0;
  /** Creates a new ExampleSubsystem. */
  public Elevator() {

    MotionMagicLibrary.setMotionMagicMotorParameters(elevatorMotor1, 0.8, 0.0, 0.0, 0.2, 10000, 10000);
    elevatorMotor1.setInverted(false);
    elevatorMotor2.setInverted(true);
  }

  public void zeroPosition() {
    elevatorMotor1.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
  }

  public void motionMagicElevator(double position) {
    // System.out.println("run");
    targetPos = position;
    elevatorMotor1.set(ControlMode.MotionMagic, targetPos);
    elevatorMotor2.follow(elevatorMotor1);
  }

  public void runElevator(double speed) {
    elevatorMotor1.set(ControlMode.PercentOutput, speed);
    elevatorMotor2.follow(elevatorMotor1);
  }

  public void printPosition() {
    System.out.println("Elevator Position: " + elevatorMotor1.getSelectedSensorPosition());
  }

  public double getVelocity() {
    return elevatorMotor1.getSelectedSensorVelocity();
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
