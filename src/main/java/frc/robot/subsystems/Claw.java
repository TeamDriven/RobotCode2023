// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.motionMagicLibrary;
import frc.robot.Constants.MotionMagicConstants;

public class Claw extends SubsystemBase {
  public TalonFX _talon = new TalonFX(10);

  public Claw() {
    //motionMagicLibrary.setMotionMagicMotorParameters(clawMotor);
    final double kP = 0.8;
    	final double kI = 0.0;
    	final double kD = 0.0;
    	final double kF = 0.2;

  		/** Creates a new ExampleSubsystem. */
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, MotionMagicConstants.kPIDLoopIdx, MotionMagicConstants.kTimeoutMs);

    	_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute,  MotionMagicConstants.kPIDLoopIdx, MotionMagicConstants.kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, MotionMagicConstants.kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, MotionMagicConstants.kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, MotionMagicConstants.kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, MotionMagicConstants.kTimeoutMs);
		_talon.configNominalOutputReverse(0, MotionMagicConstants.kTimeoutMs);
		_talon.configPeakOutputForward(0.3, MotionMagicConstants.kTimeoutMs);
		_talon.configPeakOutputReverse(-0.3, MotionMagicConstants.kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(MotionMagicConstants.kSlotIdx, MotionMagicConstants.kPIDLoopIdx);
		_talon.config_kF(MotionMagicConstants.kSlotIdx, kF, MotionMagicConstants.kTimeoutMs);
		_talon.config_kP(MotionMagicConstants.kSlotIdx, kP, MotionMagicConstants.kTimeoutMs);
		_talon.config_kI(MotionMagicConstants.kSlotIdx, kI, MotionMagicConstants.kTimeoutMs);
		_talon.config_kD(MotionMagicConstants.kSlotIdx, kD, MotionMagicConstants.kTimeoutMs);

    	/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(3000, MotionMagicConstants.kTimeoutMs);
		_talon.configMotionAcceleration(3000, MotionMagicConstants.kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, MotionMagicConstants.kPIDLoopIdx, MotionMagicConstants.kTimeoutMs);
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
