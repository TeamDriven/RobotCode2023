// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.hal.util.HalHandleException;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public VictorSPX intakeRollers = new VictorSPX(16);
  // pdp slot 13
  private final PowerDistribution m_pdp;
  
  /** Creates a new ExampleSubsystem. */
  public Intake(PowerDistribution pdp) {
    m_pdp = pdp;
    intakeRollers.setNeutralMode(NeutralMode.Brake);
    intakeRollers.setInverted(true);
  }

  public void runIntake(double speed) {
    // System.out.println("SpinWheels");
    intakeRollers.set(VictorSPXControlMode.PercentOutput, -speed);
    // intakeRollers.set(speed);
    
  }

  public double getCurrentDraw() {
    try {
      return m_pdp.getCurrent(13);
    } catch (HalHandleException e) {
      System.out.println("Can't find current");
      return 0.0;
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println(getCurrentDraw());
    //getCurrentDraw();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
