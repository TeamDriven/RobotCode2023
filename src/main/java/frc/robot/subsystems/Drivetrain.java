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
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
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
import frc.robot.commands.drivetrain.FindPath;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static frc.robot.Constants.DrivetrainConstants.*;

public class Drivetrain extends SubsystemBase {
  
  public double kMaxSpeed = 3; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI*4; // 1/2 rotation per second
  private static final double kMaxAngularAcceleration = 8 * Math.PI; // radians per second squared
  
  private final Translation2d m_frontLeftLocation = new Translation2d(0.273, 0.33);
  private final Translation2d m_frontRightLocation = new Translation2d(0.273, -0.33);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.273, 0.33);
  private final Translation2d m_backRightLocation = new Translation2d(-0.273, -0.33);

  // private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 1, 7.26186181491516);
  // private final SwerveModule m_frontRight = new SwerveModule(1, 2, 2, 1.1546138688759355);
  // private final SwerveModule m_backLeft = new SwerveModule(5, 6, 0, 5.775713308823037);
  // private final SwerveModule m_backRight = new SwerveModule(7, 8, 3, 1.207007467506754);
  private final SwerveModule m_frontLeft = new SwerveModule(3, 4, 1, 7.258332235480612);
  private final SwerveModule m_frontRight = new SwerveModule(1, 2, 2, 1.1099677520818219);
  private final SwerveModule m_backLeft = new SwerveModule(5, 6, 0, 5.694360665467085);
  private final SwerveModule m_backRight = new SwerveModule(7, 8, 3, 1.1582267005178846);

  private final static PigeonIMU m_pigey = new PigeonIMU(12);

  private final static double startingPitch = m_pigey.getPitch();
  private final static double startingRoll = m_pigey.getRoll();
  private final static double startingYaw = m_pigey.getYaw();

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
  //Changed Variables

  // HolonomicDriveController pathController = new HolonomicDriveController(
  //   new PIDController(kPXController, 0, 0),
  //   new PIDController(kPYController, 0, 0),
  //   new ProfiledPIDController(kPThetaController, 0, 0, 
  //     new TrapezoidProfile.Constraints(kMaxAngularSpeed, BLDriveMotorChannel)));

  PIDController m_xController = new PIDController(kPXController, 0, 0);
  PIDController m_yController = new PIDController(kPYController, 0, 0);
  // ProfiledPIDController m_thetaController = new ProfiledPIDController(kPThetaController, 0, 0, 
  // new TrapezoidProfile.Constraints(kMaxAngularSpeed, BLDriveMotorChannel));
  PIDController m_thetaController = new PIDController(kPThetaController, 0, 0);

  public Drivetrain() {
    m_startYaw = new Rotation2d();
    m_pigey.setFusedHeading(0);

    swerveModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    // m_thetaController.enableContinuousInput(-Math.PI, Math.PI);
  }

  public void rememberStartingPosition() {
    m_startYaw = Rotation2d.fromDegrees(m_pigey.getYaw());
  
  }
  public double getYaw(){
    // System.out.println("Pitch: " + m_pigey.getPitch());
    // System.out.println("Yaw: " + m_pigey.getYaw());
    // System.out.println("Roll: " + m_pigey.getRoll());
    return m_pigey.getYaw() - startingYaw;
  }

  public double getRoll(){
    return m_pigey.getRoll() - startingRoll;
  }

  public double getPitch() {
    return m_pigey.getPitch() - startingPitch;
  }

// public void zeroRoll(){
//   m_pigey.
// }


 


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

  public void resetOdometry(Pose2d initialPose) {
    m_odometry.resetPosition(
      Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
      new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_backLeft.getPosition(),
        m_backRight.getPosition()
      }, 
      initialPose);
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

    // System.out.println(m_odometry.getPoseMeters());
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
    // System.out.println(m_odometry.getPoseMeters());

    // for (SwerveModuleState s : swerveModuleStates) {
    //   System.out.println(s);
    // }

    this.swerveModuleStates = swerveModuleStates;

    moveSwerve();
  }

  public void boxWheels() {
    var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, 0.5));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 0);
    this.swerveModuleStates = swerveModuleStates;
    // SwerveModuleState[] swerveModuleStates = {
    //   new SwerveModuleState(0, new Rotation2d(45 / 360 * Math.PI * 2)),
    //   new SwerveModuleState(0, new Rotation2d(-45 / 360 * Math.PI * 2)),
    //   new SwerveModuleState(0, new Rotation2d(45 / 360 * Math.PI * 2)),
    //   new SwerveModuleState(0, new Rotation2d(-45 / 360 * Math.PI * 2))
    // };
    // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, kMaxSpeed);
    // this.swerveModuleStates = swerveModuleStates;
  }

  public void moveSwerve() {
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
    updateOdometry();
  }

  //public void boxWheels() {
    //m_frontLeft.Rotation2d(angleDeg:45);
    //m_frontRight.Rotation2d(angleDeg:-45);
    //m_backRight.Rotation2d(angleDeg:45);
    //m_backLeft.Rotation2d(angleDeg:-45);    

  //}

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

  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, String trajectoryFileName) {
    
    // final Trajectory trajectory = generateTrajectory(waypoints);
    // final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3.5, 3   );
    final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3, 3);
    // System.out.println(trajectory);
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
          Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          }, 
          initialPose);
      }
      // m_xController.reset();
      // m_yController.reset();
      // m_thetaController.reset();
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
    // .andThen(new FindPath(
    //   trajectory,
    //   () -> getPose(),
    //   m_kinematics,
    //   m_xController,
    //   m_yController,
    //   m_thetaController,
    //   (SwerveModuleState[] moduleStates) -> {
    //     drive(moduleStates);
    //   },
    //   this
    // )).andThen(() -> drive(0.0, 0.0, 0.0, true), this);
  }

  public SequentialCommandGroup followPathCommand(final boolean shouldResetOdometry, PathPlannerTrajectory trajectory) {
    
    // final Trajectory trajectory = generateTrajectory(waypoints);
    // final PathPlannerTrajectory trajectory = PathPlanner.loadPath(trajectoryFileName, 3.5, 3   );
    // System.out.println(trajectory);
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
          Rotation2d.fromDegrees(m_pigey.getFusedHeading()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          }, 
          initialPose);
      }
      // m_xController.reset();
      // m_yController.reset();
      // m_thetaController.reset();
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
    // .andThen(new FindPath(
    //   trajectory,
    //   () -> getPose(),
    //   m_kinematics,
    //   m_xController,
    //   m_yController,
    //   m_thetaController,
    //   (SwerveModuleState[] moduleStates) -> {
    //     drive(moduleStates);
    //   },
    //   this
    // )).andThen(() -> drive(0.0, 0.0, 0.0, true), this);
  }

  public void setMaxSpeed(double speed) {
    kMaxSpeed = speed;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // updateOdometry();

    moveSwerve();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public class SwerveModule {
    // double COUNTS_PER_METER = 51213;
    double COUNTS_PER_METER = 57032;
 // private static final double kWheelRadius = 0.0508;
//  private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Drivetrain.kMaxAngularSpeed;
  private static final double kModuleMaxAngularAcceleration = Drivetrain.kMaxAngularAcceleration;

  private final TalonFX m_driveMotor;
  private final TalonFX m_turningMotor;

  private final double offSet;
  
  private final DutyCycleEncoder m_turningEncoder;
 

  // Gains are for example purposes only - must be determined for your own robot!
  private PIDController m_drivePIDController = new PIDController(0.15, .001, 0);

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

    m_driveMotor.configFactoryDefault();
    m_turningMotor.configFactoryDefault();

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

  // public void setDrivePIDController(double kp, double ki, double kd) {
  //   m_drivePIDController = new PIDController(kp, ki, kd);
  // }

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
    return new SwerveModulePosition(m_driveMotor.getSelectedSensorPosition() / COUNTS_PER_METER, new Rotation2d(m_turningEncoder.get() * Math.PI * 2 - offSet));
    // return new SwerveModulePosition(m_driveMotor.getSelectedSensorVelocity() / COUNTS_PER_METER, new Rotation2d(m_turningEncoder.get()));
  }

  public void printData() {
    System.out.println("Pos: " + m_driveMotor.getSelectedSensorPosition() + ", Vel: " + m_driveMotor.getSelectedSensorVelocity() + ", Rot: " + m_turningEncoder.get());
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
    System.out.println(label + ": " + m_turningEncoder.getDistance()*Math.PI*2);
  }
  }
}
