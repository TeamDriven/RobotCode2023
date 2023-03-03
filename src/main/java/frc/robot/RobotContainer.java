// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
import static frc.robot.Controls.*;

import java.util.HashMap;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import frc.robot.commands.AutoMoveElevatorAndClaw;

// import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

import frc.robot.commands.RunTempIntake;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.DriveUp;
import frc.robot.commands.auto.PlaceCone;
import frc.robot.commands.auto.TestPath;
import frc.robot.commands.auto.balanceauto;
import frc.robot.commands.claw.ResetClawPosition;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.SetClawPositionWaitForFinish;
import frc.robot.commands.claw.detectSensor;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.ChangeMaxSpeed;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.intake.SpinIntakeParallel;
import frc.robot.commands.limelight.MoveTo2DAprilTags;
import frc.robot.commands.limelight.MoveToRetroreflectiveTape;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.OldIntake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Intake;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Claw m_claw = new Claw();
  private final ClawPneumatics m_ClawPneumatics = new ClawPneumatics();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LimeLight m_limelight = new LimeLight();
  //private final motionMagicMotor m_motionMagicMotor = new motionMagicMotor();
  private final Elevator m_elevator = new Elevator();
  private final OldIntake m_oldIntake = new OldIntake();
  private final Intake m_intake = new Intake();

  private final DriveUp m_DriveUp = new DriveUp(m_drivetrain);
  private final DriveForward m_DriveForward = new DriveForward(m_drivetrain);
  private final TestPath m_TestPath = new TestPath(m_drivetrain);
  private final balanceauto m_balanceauto = new balanceauto(m_drivetrain);
  private final PlaceCone m_PlaceCone = new PlaceCone(m_drivetrain, m_intake, m_elevator, m_claw);

  private boolean coneMode = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putBoolean("Cone Mode", coneMode);
  
    m_drivetrain.setDefaultCommand(new DriveContinous(m_drivetrain));
  }

  public void testEncoder() {
    m_drivetrain.printEncoders();
    // m_drivetrain.drive(0, 0, 0, false);
  }

  public void changeClawMode(NeutralMode mode) {
    m_claw.changeMode(mode);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    //Limelight

    /*new Trigger(m_controller::getYButton).whileTrue(new ParallelRaceGroup(
      new read2DAprilTags(m_limelight), 
      new MoveToLimelight(m_drivetrain, m_limelight)));
      */
    // new Trigger(m_controller::getYButton).whileTrue(new read2DAprilTags(m_limelight));
    // new Trigger(m_controller::getYButton).whileTrue(new MoveToLimelight(m_drivetrain, m_limelight));
    // new Trigger(m_controller::getYButton).whileTrue(new read3DAprilTags(m_limelight));

    /*new Trigger(m_controller::getXButton).whileTrue(new ParallelRaceGroup(
      new readRetroreflectiveTape(m_limelight), 
      new MoveToLimelight(m_drivetrain, m_limelight)));
      */
    // new Trigger(m_controller::getXButton).whileTrue(new readRetroreflectiveTape(m_limelight));
    // new Trigger(m_controller::getXButton).whileTrue(new MoveToLimelight(m_drivetrain, m_limelight));

    new Trigger(resetDriveOrientationControl)
      .onTrue(new InstantCommand(m_drivetrain::resetPidgey));

    new Trigger(resetElevatorAndClawPositionControl)
      .onTrue(new ResetElevatorPosition(m_elevator))
      .onTrue(new ResetClawPosition(m_claw));

    //Intake
    new Trigger(intakeControl)
      .and(this::isConeMode)
        .whileTrue(new RunTempIntake(m_intake, kIntakeSpeed));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
        .whileTrue(new RunTempIntake(m_intake, -kIntakeSpeed));

    new Trigger(intakeControl)
      .and(this::isCubeMode)
        .whileTrue(new RunTempIntake(m_intake, -kIntakeSpeed));
  
    new Trigger(outtakeControl)
      .and(this::isCubeMode)
        .whileTrue(new RunTempIntake(m_intake, kIntakeSpeed));

    //Elevator
    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorConeUpPos, armHighPlaceConePos));

    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorConeMidPos, armMidPlaceConePos)); 

    new Trigger(autoPickUpControl)
      .and(this::isConeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorPickUpConePos, armConePickupPos));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorCubeUpPos, armHighPlaceCubePos));
  
    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorCubeMidPos, armMidPlaceCubePos)); 
  
    new Trigger(autoPickUpControl)
      .and(this::isCubeMode)
        .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorPickUpCubePos, armCubePickupPos));

    new Trigger(autoTuckControl)
      .onTrue(new AutoMoveElevatorAndClaw(m_elevator, m_claw, elevatorTicksPerInches, armStartPos));

    new Trigger(speedUpControl)
      .whileTrue(new ChangeMaxSpeed(m_drivetrain, 30))
      .onFalse(new ChangeMaxSpeed(m_drivetrain, 8));

    new Trigger(changeModeControl)
      .onTrue(new InstantCommand(this::changeMode));
    
    // new Trigger(this::isFast)
    //   .and(m_controller::getStartButton)
    //     // .onTrue(new PrintCommand("isFast"))
    //     // .onTrue(new ChangeMaxSpeed(m_drivetrain, 3))
    //     // .onFalse(new InstantCommand(this::changeSpeed));
    //   // .onFalse(new ChangeDrivePID(m_drivetrain, 1.0, 0.001, 0));
    // //   .onTrue(new AutoPlaceConeHigh(m_intake, m_elevator, m_claw, m_ClawPneumatics));
    
    // new Trigger(this::isSlow)
    //   .and(m_controller::getStartButton)
    //     // .onTrue(new PrintCommand("isSlow"))
    //     // .onTrue(new ChangeMaxSpeed(m_drivetrain, 10))
    //     // .onFalse(new InstantCommand(this::changeSpeed));

    // new Trigger(Controls::getLeftTrigger)
    //   .whileTrue(new MoveToRetroreflectiveTape(m_limelight, m_drivetrain)); 

    // new Trigger(Controls::getRightTrigger)
    //   .whileTrue(new MoveTo2DAprilTags(m_limelight, m_drivetrain)); 

    new Trigger(autoBalanceControl)
      // .whileTrue(new PrintCommand("Box wheels"))
      // .whileTrue(new InstantCommand(m_drivetrain::boxWheels));
      .whileTrue(new AutoBalance(m_drivetrain));

    new Trigger(moveElevatorUpControl)
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      .whileTrue(new RunElevator(m_elevator, 0.5)) // change
      .onFalse(new RunElevator(m_elevator, 0.0));
    
    new Trigger(moveElevatorDownControl)
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      .whileTrue(new RunElevator(m_elevator, -.5)) // change
      .onFalse(new RunElevator(m_elevator, 0.0));

      //Test Intake
      new Trigger(moveClawUpControl)
        // .whileTrue(new InstantCommand(m_claw::printPosition))
        .whileTrue(new InstantCommand(m_claw::runMotorForward))
        .onFalse(new InstantCommand(m_claw::stopMotor));

      
      new Trigger(moveClawDownControl)
        // .whileTrue(new InstantCommand(m_claw::printPosition))
        .whileTrue(new InstantCommand(m_claw::runMotorBackward))
        .onFalse(new InstantCommand(m_claw::stopMotor));

  
 
}

  public void changeMode() {
    coneMode = !coneMode;
    SmartDashboard.putBoolean("Cone Mode", coneMode);
  }

  public boolean isConeMode() {
    return coneMode;
  }

  public boolean isCubeMode() {
    return !coneMode;
  }

  // public void printPOV() {
  //   System.out.println(m_controller.getPOV());
  // }

  public boolean getNotLeftBumper() {
    if (!m_controller.getLeftBumper()) {
      return true;
    } else {
      return false;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // final PathPlannerTrajectory trajectory = PathPlanner.loadPath("test", 3, 3);
    // HashMap<String, Command> eventMap = new HashMap<>();
    // eventMap.put("Mark1", new PrintCommand("Mark 1"));
    
    // return new FollowPathWithEvents(
    //   m_drivetrain.followPathCommand(true, trajectory),
    //   trajectory.getMarkers(),
    //   eventMap
    // );
    // An example command will be run in autonomous
    return m_PlaceCone;
    // return Autos.exampleAuto();
  }
}
