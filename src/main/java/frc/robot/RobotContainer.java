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
import frc.robot.commands.automation.PickUpSubstationCone;
import frc.robot.commands.automation.PlaceConeHighTeleOp;
import frc.robot.commands.automation.PlaceConeMidTeleOp;
import frc.robot.commands.automation.PlaceCubeHighAuto;
import frc.robot.commands.automation.PlaceCubeHighTeleOp;
import frc.robot.commands.automation.PlaceCubeMidTeleOp;
import frc.robot.commands.automation.ZeroElevatorAndClaw;
import frc.robot.commands.autonomous.PlaceConeAuto;
import frc.robot.commands.autonomous.PlaceConeBalanceAuto;
import frc.robot.commands.autonomous.PlaceConeMobilityAuto;
import frc.robot.commands.autonomous.ThreePlaceTopBlue;
import frc.robot.commands.autonomous.ThreePlaceTopRed;
import frc.robot.commands.autonomous.TwoPlaceParkTopBlue;
import frc.robot.commands.autonomous.TwoPlaceParkTopRed;
import frc.robot.commands.autonomous.TwoPlaceTopBlue;
import frc.robot.commands.autonomous.TwoPlaceTopRed;
import frc.robot.commands.autonomous.balanceauto;
import frc.robot.commands.claw.AutoResetClawPosition;
import frc.robot.commands.claw.ResetClawPosition;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.AutoTurnToSubstation;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.ChangeMaxSpeed;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.drivetrain.SprintDrive;
import frc.robot.commands.elevator.AutoResetElevatorPosition;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.limelight.MoveTo2DAprilTags;
import frc.robot.commands.limelight.MoveToRetroreflectiveTape;
import frc.robot.commands.limelight.readRetroreflectiveTape;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Intake;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
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
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LimeLight m_limelight = new LimeLight();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final balanceauto m_balanceauto = new balanceauto(m_drivetrain);
  private final PlaceConeAuto m_PlaceConeAuto = new PlaceConeAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeBalanceAuto m_PlaceConeBalanceAuto = new PlaceConeBalanceAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeMobilityAuto m_PlaceConeMobilityAuto = new PlaceConeMobilityAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final TwoPlaceParkTopBlue m_TwoPlaceParkTopBlue = new TwoPlaceParkTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceParkTopRed m_TwoPlaceParkTopRed = new TwoPlaceParkTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceTopBlue m_TwoPlaceTopBlue = new TwoPlaceTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceTopRed m_TwoPlaceTopRed = new TwoPlaceTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final ThreePlaceTopBlue m_ThreePlaceTopBlue = new ThreePlaceTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final ThreePlaceTopRed m_ThreePlaceTopRed = new ThreePlaceTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);

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
    m_chooser.addOption("Two Place Blue", m_TwoPlaceTopBlue);
    m_chooser.addOption("Two Place Red", m_TwoPlaceTopRed);
    m_chooser.addOption("Place and mobility", m_PlaceConeMobilityAuto);
    m_chooser.addOption("Place and balance", m_PlaceConeBalanceAuto);
    m_chooser.addOption("Place only", m_PlaceConeAuto);
    m_chooser.addOption("Balance only", m_balanceauto);
    m_chooser.addOption("Place Three Blue", m_ThreePlaceTopBlue);
    m_chooser.addOption("Place Three Red", m_ThreePlaceTopRed);
    SmartDashboard.putData(m_chooser);
  }

  public void testEncoder() {
    m_drivetrain.printEncoders();
    // m_drivetrain.drive(0, 0, 0, false);
  }

  public void boxWheels() {
    m_drivetrain.boxWheels();
  }

  public void changeClawMode(NeutralMode mode) {
    m_claw.changeMode(mode);
  }

  public void changeOffset(double offset) {
    m_drivetrain.adjustOffset(offset);
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
      .onTrue(new ZeroElevatorAndClaw(m_elevator, m_claw));
      
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
        .whileTrue(new RunTempIntake(m_intake, 0.5));

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceConeHighTeleOp(m_elevator, m_claw, m_drivetrain));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceCubeHighTeleOp(m_elevator, m_claw, m_intake, m_drivetrain));

    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new PlaceConeMidTeleOp(m_elevator, m_claw, m_drivetrain)); 

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new PlaceCubeMidTeleOp(m_elevator, m_claw, m_intake, m_drivetrain)); 

    new Trigger(autoFloorConePickUpControl)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpConePos, armConePickupPos));

    // if (DriverStation.getAlliance().equals(Alliance.Blue)) {
    new Trigger(autoPickUpControl)
      .and(this::isConeMode)
        // .onTrue(new InstantCommand(this::setToPickUpPos))
        // .whileTrue(new PickUpSubstationCone(m_drivetrain, m_intake, m_elevator, m_claw));
          // .onTrue(new InstantCommand(this::putAllianceColor));
          .onTrue(new ParallelCommandGroup(
            new InstantCommand(this::setToPickUpPos),
            new RunTempIntake(m_intake, kIntakeSpeed),
            new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorSubstationPos, armSubstationPos)
          ))
          .whileTrue(new AutoTurnToSubstation(m_drivetrain));
    // } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
    //   new Trigger(autoPickUpControl)
    //     .and(this::isConeMode)
    //       .onTrue(new ParallelCommandGroup(
    //         new InstantCommand(this::setToPickUpPos),
    //         new RunTempIntake(m_intake, kIntakeSpeed),
    //         new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorSubstationPos, armSubstationPos)
    //       ))
    //       .whileTrue(new AutoTurn(m_drivetrain, -90));
    // }

  
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

    new Trigger(autoTuckControl)
      .and(this::isConeMode)
        .onTrue(new RunTempIntake(m_intake, 0.2));

    new Trigger(speedUpControl)
      .onTrue(new ChangeMaxSpeed(m_drivetrain, kFastDriveSpeed))
      .whileTrue(new SprintDrive(m_drivetrain))
      .onFalse(new ChangeMaxSpeed(m_drivetrain, kSlowDriveSpeed));

    new Trigger(changeModeControl)
      .whileTrue(new RunTempIntake(m_intake, 0.0))
      .onTrue(new InstantCommand(this::changeMode));

    
    new Trigger(placeConeOnPoleControl)
      .and(this::getIsPlaceHighPos)
        .onTrue(new SetClawPosition(m_claw, armOnHighPole));
    
    new Trigger(placeConeOnPoleControl)
      .and(this::getIsPlaceMidPos)
        .onTrue(new SetClawPosition(m_claw, armOnMidPole));

    new Trigger(autoLineUpControl)
      .and(this::isConeMode)
        .whileTrue(new SequentialCommandGroup(
          new AutoTurn(m_drivetrain, 180),
          new MoveToRetroreflectiveTape(m_limelight, m_drivetrain)
        ));
        // .whileTrue(new MoveToRetroreflectiveTape(m_limelight, m_drivetrain));
    
    new Trigger(autoLineUpControl)
      .and(this::isCubeMode)
        .whileTrue(new SequentialCommandGroup(
          new AutoTurn(m_drivetrain, 180),
          new MoveTo2DAprilTags(m_limelight, m_drivetrain)
        ));
        // .whileTrue(new MoveTo2DAprilTags(m_limelight, m_drivetrain));

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

  public void putAllianceColor() {
    System.out.println(DriverStation.getAlliance().name());
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
    // return new SequentialCommandGroup(
    //   new ParallelDeadlineGroup(
    //     new WaitCommand(14.75), 
    //     m_chooser.getSelected()
    //   ),
    //   new BoxWheels(m_drivetrain)
    // );
    return m_chooser.getSelected();
    // return new PlaceCubeHighAuto(m_elevator, m_claw, m_intake);
  }
}
