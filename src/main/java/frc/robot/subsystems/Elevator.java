// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  VictorSPX elevatorMotor1 = new VictorSPX(13);
  VictorSPX elevatorMotor2 = new VictorSPX(14);

  /** Creates a new ExampleSubsystem. */
  public Elevator() {
    elevatorMotor1.setInverted(true);
    elevatorMotor2.setInverted(false);
  }

  public void moveElevator(double speed) {
    elevatorMotor1.set(VictorSPXControlMode.PercentOutput, speed);
    elevatorMotor2.set(VictorSPXControlMode.PercentOutput, speed);
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
