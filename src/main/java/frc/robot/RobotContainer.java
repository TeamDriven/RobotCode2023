// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Controls.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.commands.RunTempIntake;
import frc.robot.commands.automation.MoveElevatorAndClaw;
import frc.robot.commands.automation.MoveElevatorAndClawFast;
import frc.robot.commands.automation.PlaceConeHighTeleOp;
import frc.robot.commands.automation.PlaceCubeHighTeleOp;
import frc.robot.commands.automation.PlaceCubeMidTeleOp;
import frc.robot.commands.autonomous.TwoPlaceParkTopBlue;
import frc.robot.commands.autonomous.TwoPlaceParkTopRed;
import frc.robot.commands.autonomous.balanceauto;
import frc.robot.commands.claw.ResetClawPosition;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.drivetrain.ChangeMaxSpeed;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.limelight.MoveToRetroreflectiveTape;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LimeLight m_limelight = new LimeLight();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final balanceauto m_balanceauto = new balanceauto(m_drivetrain);
  private final TwoPlaceParkTopBlue m_TwoPlaceParkTopBlue = new TwoPlaceParkTopBlue(m_drivetrain, m_intake, m_elevator, m_claw);
  private final TwoPlaceParkTopRed m_TwoPlaceParkTopRed = new TwoPlaceParkTopRed(m_drivetrain, m_intake, m_elevator, m_claw);

  private boolean coneMode = true;

  private boolean isTuckPos = true;
  private boolean isPickUpPos = false;
  private boolean isPlaceMidPos = false;
  private boolean isPlaceHighPos = false;

  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    SmartDashboard.putBoolean("Cone Mode", coneMode);
  
    m_drivetrain.setDefaultCommand(new DriveContinous(m_drivetrain));

    m_chooser.setDefaultOption("Two Place And Balance Blue", m_TwoPlaceParkTopBlue);
    m_chooser.addOption("Two Place And Balance Red", m_TwoPlaceParkTopRed);
    m_chooser.addOption("Balance only", m_balanceauto);
    SmartDashboard.putData(m_chooser);
  }

  public void testEncoder() {
    m_drivetrain.printEncoders();
  }

  public void boxWheels() {
    m_drivetrain.boxWheels();
  }

  public void changeClawMode(NeutralMode mode) {
    m_claw.changeMode(mode);
  }

  public void changeOffset(double offset) {
    m_drivetrain.setOffset(offset);
  }

  public void changeDrivePIDController(PIDController drivePID) {
    m_drivetrain.setDrivePID(drivePID);
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

    new Trigger(zeroRobotControl)
      .onTrue(new InstantCommand(m_drivetrain::resetPidgey))
      .onTrue(new ResetElevatorPosition(m_elevator))
      .onTrue(new ResetClawPosition(m_claw));
      
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

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceConeHighTeleOp(m_elevator, m_claw));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceCubeHighTeleOp(m_elevator, m_claw, m_intake));

    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorConeMidPos, armMidPlaceConePos)); 

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new PlaceCubeMidTeleOp(m_elevator, m_claw, m_intake)); 

    new Trigger(autoFloorConePickUpControl)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpConePos, armConePickupPos));

    new Trigger(autoPickUpControl)
      .and(this::isConeMode)
        .onTrue(new ParallelCommandGroup(
          new InstantCommand(this::setToPickUpPos),
          new RunTempIntake(m_intake, kIntakeSpeed),
          new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorSubstationPos, armSubstationPos)
        ));
  
    new Trigger(autoPickUpControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpCubePos, armCubePickupPos));

    new Trigger(autoTuckControl)
      .and(this::getIsPickUpPos)
        .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(autoTuckControl)
      .and(this::getIsNotPickUpPos)
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(speedUpControl)
      .whileTrue(new ChangeMaxSpeed(m_drivetrain, kFastDriveSpeed))
      .onFalse(new ChangeMaxSpeed(m_drivetrain, kSlowDriveSpeed));

    new Trigger(changeModeControl)
      .onTrue(new InstantCommand(this::changeMode));

    
    new Trigger(placeConeOnPoleControl)
      .and(this::getIsPlaceHighPos)
        .onTrue(new SetClawPosition(m_claw, armOnHighPole));
    
    new Trigger(placeConeOnPoleControl)
      .and(this::getIsPlaceMidPos)
        .onTrue(new SetClawPosition(m_claw, armOnMidPole));

    new Trigger(autoLineUpControl)
      .whileTrue(new MoveToRetroreflectiveTape(m_limelight, m_drivetrain)); 

    new Trigger(moveElevatorUpControl)
      .whileTrue(new RunElevator(m_elevator, 0.5))
      .onFalse(new RunElevator(m_elevator, 0.0));
    
    new Trigger(moveElevatorDownControl)
      .whileTrue(new RunElevator(m_elevator, -.5))
      .onFalse(new RunElevator(m_elevator, 0.0));

    new Trigger(moveClawUpControl)
      .whileTrue(new InstantCommand(m_claw::runMotorForward))
      .onFalse(new InstantCommand(m_claw::stopMotor));

      
    new Trigger(moveClawDownControl)
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

  
  public void setToTuckPos() {
    isTuckPos = true;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
  }

  public void setToPickUpPos() {
    isTuckPos = false;
    isPickUpPos = true;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
  }

  public void setToPlaceMidPos() {
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = true;
    isPlaceHighPos = false;
  }

  public void setToPlaceHighPos() {
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = true;
  }


  public boolean getIsPickUpPos() {
    return isPickUpPos;
  }

  public boolean getIsNotPickUpPos() {
    return !isPickUpPos;
  }

  public boolean getIsPlaceHighPos() {
    return isPlaceHighPos;
  }

  public boolean getIsNotPlaceHighPos() {
    return !isPlaceHighPos;
  }

  public boolean getIsPlaceMidPos() {
    return isPlaceMidPos;
  }

  public boolean getIsNotPlaceMidPos() {
    return !isPlaceMidPos;
  }

  public boolean getIsTuckPos() {
    return isTuckPos;
  }

  public boolean getIsNotTuckPos() {
    return !isTuckPos;
  }

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
    return m_chooser.getSelected();
  }
}
