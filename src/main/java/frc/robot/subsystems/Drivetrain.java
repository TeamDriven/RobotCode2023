// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;;

public class Drivetrain extends SubsystemBase {
  
  public static final double kMaxSpeed = 3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI*4; // 1/2 rotation per second

  private final Translation2d m_frontLeftLocation = new Translation2d(0.267, 0.267);
  private final Translation2d m_frontRightLocation = new Translation2d(0.267, -0.267);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.267, 0.267);
  private final Translation2d m_backRightLocation = new Translation2d(-0.267, -0.267);

  private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 1, 5.649738110618463);
  private final SwerveModule m_frontRight = new SwerveModule(5, 6, 2, 0.9418667305466683+3.14);
  private final SwerveModule m_backLeft = new SwerveModule(1, 2, 0, 0.16685627318886792);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 3, -0.4638748593756933);

  private final static PigeonIMU m_pigey = new PigeonIMU(11);

  private final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  private final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
        m_kinematics,
        Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });

  private Rotation2d m_headingAdjust = new Rotation2d();
  private double m_rollOffset;
  private Rotation2d m_startYaw;
  private SwerveModuleState[] swerveModuleStates;
  
  private static final PIDController m_xController = new PIDController(DrivetrainConstants.PX_CONTROLLER, 0, 0);
  private static final PIDController m_yController = new PIDController(DrivetrainConstants.PY_CONTROLLER, 0, 0);
  private static final PIDController m_thetaController = new PIDController(DrivetrainConstants.PTHETA_CONTROLLER, 0, 0);

  public Drivetrain() {
    m_startYaw = new Rotation2d();
    m_pigey.setFusedHeading(0);

    swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

    m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void rememberStartingPosition() {
    m_startYaw = Rotation2d.fromDegrees(m_pigey.getYaw());
  }

  public void reZeroFromStartingPositon() {
    setHeadingAdjust(Rotation2d.fromDegrees(m_pigey.getCompassHeading()).minus(m_startYaw));
  }

  public Rotation2d getHeadingAdjust() {
    return m_headingAdjust;
  }

  private void setHeadingAdjust(final Rotation2d headingAdjust) {
    m_headingAdjust = headingAdjust;
    // System.out.println("Set heading adjust: " + m_headingAdjust.getDegrees());
  }

  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot,  Rotation2d.fromDegrees(m_pigey.getFusedHeading()))
                : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    this.swerveModuleStates = swerveModuleStates;
    // m_frontLeft.setDesiredState(swerveModuleStates[0]);
    // m_frontRight.setDesiredState(swerveModuleStates[1]);
    // m_backLeft.setDesiredState(swerveModuleStates[2]);
    // m_backRight.setDesiredState(swerveModuleStates[3]);
    //frontright 0.15214992880374822
    //frontleft -0.07802162695054071
    //backleft 0.0
    //backright -0.07353397683834939
    // m_backLeft.printencoder("bl");
    //m_backRight.printencoder("br");
    //m_frontLeft.printencoder("fl");
    //m_frontRight.printencoder("fr");

    //System.out.println(Rotation2d.fromDegrees(m_pigey.getFusedHeading()));

    moveSwerve();
  }

  public void drive(SwerveModuleState[] swerveModuleStates) {
    this.swerveModuleStates = swerveModuleStates;
    
    //System.out.println(Rotation2d.fromDegrees(m_pigey.getFusedHeading()));

    moveSwerve();
  }

  public void moveSwerve() {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  public void printEncoders() {
    m_backLeft.printencoder("bl");
     m_backRight.printencoder("br");
     m_frontLeft.printencoder("fl");
     m_frontRight.printencoder("fr");
  }

  public static void resetPidgey(){
    m_pigey.setFusedHeading(0);
  }
  
  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Rotation2d getAdjustedHeading() {
    // TODO compass heading only returns 180 now... why?
  //  if (m_navx.isMagnetometerCalibrated()) {
     // We will only get valid fused headings if the magnetometer is calibrated
    //  return Rotation2d.fromDegrees(m_navx.getCompassHeading()).rotateBy(getHeadingAdjust());
  //  }

   // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
   return Rotation2d.fromDegrees(360.0 - m_pigey.getYaw());
  }

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, String trajectoryFileName) {
    
    // final Trajectory trajectory = generateTrajectory(waypoints);
    final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3.5, 3   );
    // double Seconds = 0.0;
    // System.out.println("===== Begin Sampling path =====");
    // while(trajectory.getTotalTimeSeconds() > Seconds) {
    //   PathPlannerState state = (PathPlannerState) trajectory.sample(Seconds);
    //   System.out.println(
    //     "time: " + Seconds
    //     + ", x: " + state.poseMeters.getX()
    //     + ", y: " + state.poseMeters.getY()
    //     + ", angle: " + state.poseMeters.getRotation().getDegrees()
    //     + ", holo: " + state.holonomicRotation.getDegrees()
    //   );
    // Seconds += 0.25;
    // }
    // System.out.println("===== End Sampling Path =====");
    return new InstantCommand(() -> {
      if (shouldResetOdometry) {
        PathPlannerState initialSample = (PathPlannerState) trajectory.sample(0);
        Pose2d initialPose = new Pose2d(initialSample.poseMeters.getTranslation(), initialSample.holonomicRotation);
        m_odometry.resetPosition(
          getAdjustedHeading(),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          }, 
          initialPose);
      }
      m_xController.reset();
      m_yController.reset();
    }).andThen(new PPSwerveControllerCommand(
      trajectory,
      () -> getPose(),
      m_kinematics,
      m_xController,
      m_yController,
      m_thetaController,
      (SwerveModuleState[] moduleStates) -> {
        drive(moduleStates);
      },
      this
    )).andThen(() -> drive(0.0, 0.0, 0.0, true), this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    updateOdometry();

    moveSwerve();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public class SwerveModule {
    double COUNTS_PER_METER = 51213;
 // private static final double kWheelRadius = 0.0508;
//  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration =
      8 * Math.PI; // radians per second squared

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final double offSet;
  
  private final DutyCycleEncoder m_turningEncoder;
 

  // Gains are for example purposes only - must be determined for your own robot!
  private final PIDController m_drivePIDController = new PIDController(0.0001, 0, 0);

  // Gains are for example purposes only - must be determined for your own robot!
  private final ProfiledPIDController m_turningPIDController =
      new ProfiledPIDController(
          0.5,
          0,
          0,
          new TrapezoidProfile.Constraints(
              kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

  // Gains are for example purposes only - must be determined for your own robot!
  private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0.1, 0.1);
  private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

  /**
   * Constructs a SwerveModule with a drive motor, turning motor, drive encoder and turning encoder.
   *
   * @param driveMotorChannel PWM output for the drive motor.
   * @param turningMotorChannel PWM output for the turning motor.
   * @param driveEncoderChannelA DIO input for the drive encoder channel A
   * @param turningEncoderChannelA DIO input for the turning encoder channel A
   * @param turningEncoderChannelB DIO input for the turning encoder channel B
   * 
   */
  public SwerveModule(
      int driveMotorChannel,
      int turningMotorChannel,
      int turningEncoderChannel,
      double _offSet)
       {
    m_driveMotor = new TalonFX(driveMotorChannel);
    m_turningMotor = new TalonFX(turningMotorChannel);

    //m_driveEncoder = new Encoder(driveEncoderChannelA, driveEncoderChannelB);
    m_turningEncoder = new DutyCycleEncoder(turningEncoderChannel);
    offSet = _offSet;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * pi) divided by the
    // encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(m_driveMotor.getSelectedSensorVelocity() / COUNTS_PER_METER, new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Returns the current position of the module
   * 
   * @return The current position of the module
   */
  //TODO: Check to make sure this is right
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(m_driveMotor.getSelectedSensorPosition() / COUNTS_PER_METER, new Rotation2d(m_turningEncoder.get()));
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    double tempEncoderCycle = m_turningEncoder.getDistance() * Math.PI*2 -offSet;
    double encoderUpperLimit = Math.PI;
    double encoderLowerLimit = -Math.PI;

    //checking to make sure within range, and correcting
  if(tempEncoderCycle > encoderUpperLimit) {
      double difference = tempEncoderCycle - encoderUpperLimit;
      tempEncoderCycle = encoderLowerLimit + difference;
  } else if (tempEncoderCycle < encoderLowerLimit) {
      double difference = encoderLowerLimit - tempEncoderCycle;
      tempEncoderCycle = encoderUpperLimit - difference;
  }
    // Optimize the reference state to avoid spinning further than 90 degrees

    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(tempEncoderCycle));
    // Calculate the drive output from the drive PID controller.
    final double driveOutput =
        m_drivePIDController.calculate(m_driveMotor.getSelectedSensorVelocity() / COUNTS_PER_METER, state.speedMetersPerSecond);

    final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);
    //System.out.println(state.angle);
    // Calculate the turning motor output from the turning PID controller.
    final double turnOutput =
        m_turningPIDController.calculate(tempEncoderCycle,state.angle.getRadians());

    final double turnFeedforward = 0;
        //m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);
    //System.out.println(tempEncoderCycle + " : "
    //                 + desiredState.angle.getRadians() + " : " +
    //                 turnOutput);
  
    m_driveMotor.set(TalonFXControlMode.PercentOutput,driveOutput + driveFeedforward);
    m_turningMotor.set(TalonFXControlMode.PercentOutput,turnOutput + turnFeedforward);
  }
  public void printencoder(String label){
    //System.out.println(label + ": " + m_turningEncoder.getDistance()*Math.PI*2);
  }
  }
}