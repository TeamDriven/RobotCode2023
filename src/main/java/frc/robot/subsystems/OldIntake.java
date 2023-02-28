// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class OldIntake extends SubsystemBase {
  public CANSparkMax intakeHigh = new CANSparkMax(0, MotorType.kBrushless); // should be 21
  public CANSparkMax intakeLow = new CANSparkMax(23, MotorType.kBrushless);
  public DoubleSolenoid intakeCylinder = new DoubleSolenoid(30, PneumaticsModuleType.REVPH, 1, 0);
  
  /** Creates a new ExampleSubsystem. */
  public OldIntake() {
    intakeLow.restoreFactoryDefaults();
    intakeHigh.restoreFactoryDefaults();

    intakeHigh.setInverted(true);
    intakeLow.setInverted(true);
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
    // System.out.println("SpinWheels");
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

  public void intakePosition(boolean position){
    if (position) {
      intakeCylinder.set(Value.kForward);
    } else {
      intakeCylinder.set(Value.kReverse);
    }
  }

  public void resetIntakePosition() {
    intakeCylinder.set(Value.kOff);
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
