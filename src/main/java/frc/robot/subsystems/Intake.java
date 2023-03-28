// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public VictorSPX intakeRollers = new VictorSPX(16);
  
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    intakeRollers.setNeutralMode(NeutralMode.Brake);
    intakeRollers.setInverted(true);
  }

  public void runIntake(double speed) {
    // System.out.println("SpinWheels");
    intakeRollers.set(VictorSPXControlMode.PercentOutput, -speed);
    // intakeRollers.set(speed);
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
