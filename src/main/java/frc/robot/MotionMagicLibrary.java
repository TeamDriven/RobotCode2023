package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import static frc.robot.Constants.MotionMagicConstants.*;

public class MotionMagicLibrary {
	
    public static void setMotionMagicMotorParameters(WPI_TalonSRX _talon){
    	final double kP = 0.8;
    	final double kI = 0.0;
    	final double kD = 0.0;
    	final double kF = 0.2;

  		/** Creates a new ExampleSubsystem. */
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

    	_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1.0, kTimeoutMs);
		_talon.configPeakOutputReverse(-1.0, kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		_talon.config_kF(kSlotIdx, kF, kTimeoutMs);
		_talon.config_kP(kSlotIdx, kP, kTimeoutMs);
		_talon.config_kI(kSlotIdx, kI, kTimeoutMs);
		_talon.config_kD(kSlotIdx, kD, kTimeoutMs);

    	/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(3000, kTimeoutMs);
		_talon.configMotionAcceleration(3000, kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }

	public static void setMotionMagicMotorParameters(WPI_TalonSRX _talon, double kP, double kI, double kD, double kF, double velocity, double acceleration){

  		/** Creates a new ExampleSubsystem. */
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx, kTimeoutMs);

    	_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, kPIDLoopIdx, kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1.0, kTimeoutMs);
		_talon.configPeakOutputReverse(-1.0, kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		_talon.config_kF(kSlotIdx, kF, kTimeoutMs);
		_talon.config_kP(kSlotIdx, kP, kTimeoutMs);
		_talon.config_kI(kSlotIdx, kI, kTimeoutMs);
		_talon.config_kD(kSlotIdx, kD, kTimeoutMs);

    	/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(velocity, kTimeoutMs);
		_talon.configMotionAcceleration(acceleration, kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }

	public static void setMotionMagicMotorParameters(TalonFX _talon){
    	final double kP = 0.8;
    	final double kI = 0.0;
    	final double kD = 0.0;
    	final double kF = 0.2;

  		/** Creates a new ExampleSubsystem. */
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    	_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1.0, kTimeoutMs);
		_talon.configPeakOutputReverse(-1.0, kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		_talon.config_kF(kSlotIdx, kF, kTimeoutMs);
		_talon.config_kP(kSlotIdx, kP, kTimeoutMs);
		_talon.config_kI(kSlotIdx, kI, kTimeoutMs);
		_talon.config_kD(kSlotIdx, kD, kTimeoutMs);

    	/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(3000, kTimeoutMs);
		_talon.configMotionAcceleration(3000, kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }

	public static void setMotionMagicMotorParameters(TalonFX _talon, double kP, double kI, double kD, double kF, double velocity, double acceleration){

  		/** Creates a new ExampleSubsystem. */
    	_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

    	_talon.configFactoryDefault();

		/* Configure Sensor Source for Pirmary PID */
		_talon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx, kTimeoutMs);

		/* set deadband to super small 0.001 (0.1 %).
			The default deadband is 0.04 (4 %) */
		_talon.configNeutralDeadband(0.04, kTimeoutMs);

		/**
		 * Configure Talon SRX Output and Sensor direction accordingly Invert Motor to
		 * have green LEDs when driving Talon Forward / Requesting Postiive Output Phase
		 * sensor to have positive increment when driving Talon Forward (Green LED)
		 */
		_talon.setSensorPhase(false);
		_talon.setInverted(true);

		/* Set relevant frame periods to be at least as fast as periodic rate */
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10, kTimeoutMs);
		_talon.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, kTimeoutMs);

		/* Set the peak and nominal outputs */
		_talon.configNominalOutputForward(0, kTimeoutMs);
		_talon.configNominalOutputReverse(0, kTimeoutMs);
		_talon.configPeakOutputForward(1.0, kTimeoutMs);
		_talon.configPeakOutputReverse(-1.0, kTimeoutMs);

    
    
		/* Set Motion Magic gains in slot0 - see documentation */
		_talon.selectProfileSlot(kSlotIdx, kPIDLoopIdx);
		_talon.config_kF(kSlotIdx, kF, kTimeoutMs);
		_talon.config_kP(kSlotIdx, kP, kTimeoutMs);
		_talon.config_kI(kSlotIdx, kI, kTimeoutMs);
		_talon.config_kD(kSlotIdx, kD, kTimeoutMs);

    	/* Set acceleration and vcruise velocity - see documentation */
		_talon.configMotionCruiseVelocity(velocity, kTimeoutMs);
		_talon.configMotionAcceleration(acceleration, kTimeoutMs);

		/* Zero the sensor once on robot boot up */
		_talon.setSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
    }
}
