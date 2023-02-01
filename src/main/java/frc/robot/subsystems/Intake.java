// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public CANSparkMax intakeHigh = new CANSparkMax(21, MotorType.kBrushless);
  public CANSparkMax intakeLow = new CANSparkMax(23, MotorType.kBrushless);
  
  /** Creates a new ExampleSubsystem. */
  public Intake() {
    //intakeHigh.setInverted(false);
    //intakeLow.setInverted(false);
    intakeLow.restoreFactoryDefaults();
    intakeHigh.restoreFactoryDefaults();
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public CommandBase exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  public void spinWheels(double speed) {
    intakeHigh.set(speed);
    intakeLow.set(speed);
  }

  public void spinWheelsParallel(double speed) {
     intakeHigh.set(speed);
     intakeLow.set(-speed);
  }

  public void stopWheels() {
    intakeHigh.set(0);
    intakeLow.set(0);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
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
