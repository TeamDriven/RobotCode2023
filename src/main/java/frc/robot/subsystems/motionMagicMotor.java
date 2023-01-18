// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;


import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;

public class motionMagicMotor extends SubsystemBase {
    final double kP = 0.8;
    final double kI = 0.0;
    final double kD = 0.0;
    final double kF = 0.2;
    double targetPos = 0;

  WPI_TalonSRX _talon = new WPI_TalonSRX(30);
	Joystick _joy = new Joystick(0);

  /** Creates a new ExampleSubsystem. */
  public motionMagicMotor() {
    _talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
    Constants.kTimeoutMs);

    _talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, Constants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, Constants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, Constants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, Constants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, Constants.kTimeoutMs);
		_talon.configPeakOutputForward(1, Constants.kTimeoutMs);
		_talon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		_talon.config_kF(Constants.kSlotIdx, kF, Constants.kTimeoutMs);
		_talon.config_kP(Constants.kSlotIdx, kP, Constants.kTimeoutMs);
		_talon.config_kI(Constants.kSlotIdx, kI, Constants.kTimeoutMs);
		_talon.config_kD(Constants.kSlotIdx, kD, Constants.kTimeoutMs);

    /* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(3000, Constants.kTimeoutMs);
		_talon.configMotionAcceleration(3000, Constants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
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
