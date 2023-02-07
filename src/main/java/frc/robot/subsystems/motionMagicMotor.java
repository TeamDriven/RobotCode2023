// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.motionMagicLibrary;


import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants.MotionMagicConstants;

public class motionMagicMotor extends SubsystemBase {
  double targetPos = 0;

  WPI_TalonSRX _talon = new WPI_TalonSRX(30);
	Joystick _joy = new Joystick(0);

  /** Creates a new ExampleSubsystem. */
  public motionMagicMotor() {
    motionMagicLibrary.setMotionMagicMotorParameters(_talon);
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

  public void setToPosition(double position){
    //double rghtYstick = 1.0 * _joy.getRawAxis(5);
    // double targetPos = rghtYstick * 4096 * 1.0;
    targetPos = position;
    _talon.set(ControlMode.MotionMagic, targetPos);
  }

  public void printMotionMagicValues(){
     System.out.print("Talon position: " + _talon.getSelectedSensorPosition());
     System.out.print("  Motor Percentage: " + _talon.getMotorOutputPercent());
     System.out.print("  Target Position: " + targetPos);
    // System.out.println(rghtYstick);
    //_talon.set(ControlMode.PercentOutput, leftYstick);
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
