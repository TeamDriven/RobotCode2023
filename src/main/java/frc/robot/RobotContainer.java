// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
import static frc.robot.Constants.DrivetrainConstants.*;
import static frc.robot.Controls.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.commands.FlashPurple;
import frc.robot.commands.FlashYellow;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.automation.MoveElevatorAndClaw;
import frc.robot.commands.automation.MoveElevatorAndClawFast;
import frc.robot.commands.automation.PickUpSubstationCone;
import frc.robot.commands.automation.PlaceConeHighFromPrePlaceTeleOp;
import frc.robot.commands.automation.PlaceConeHighTeleOp;
import frc.robot.commands.automation.PlaceConeMidTeleOp;
import frc.robot.commands.automation.PlaceCubeHighAuto;
import frc.robot.commands.automation.PlaceCubeHighFromPrePlaceTeleOp;
import frc.robot.commands.automation.PlaceCubeHighTeleOp;
import frc.robot.commands.automation.PlaceCubeMidTeleOp;
import frc.robot.commands.automation.TuckFromPlace;
import frc.robot.commands.automation.ZeroElevatorAndClaw;
import frc.robot.commands.autonomous.PlaceConeAuto;
import frc.robot.commands.autonomous.PlaceConeBalanceAuto;
import frc.robot.commands.autonomous.PlaceConeGrabBumpAutoRed;
import frc.robot.commands.autonomous.PlaceConeGrabBumpBlueAuto;
import frc.robot.commands.autonomous.PlaceConeLeftBalanceAuto;
import frc.robot.commands.autonomous.PlaceConeMobilityAuto;
import frc.robot.commands.autonomous.PlaceConeRightBalanceAuto;
import frc.robot.commands.autonomous.PlaceCubeAuto;
import frc.robot.commands.autonomous.PlaceCubeBalanceAuto;
import frc.robot.commands.autonomous.PlaceCubeMobilityAuto;
import frc.robot.commands.autonomous.ThreePlaceTopBlue;
import frc.robot.commands.autonomous.ThreePlaceTopRed;
import frc.robot.commands.autonomous.TwoPlaceBumpBlue;
import frc.robot.commands.autonomous.TwoPlaceBumpRed;
import frc.robot.commands.autonomous.TwoPlaceParkTopBlue;
import frc.robot.commands.autonomous.TwoPlaceParkTopRed;
import frc.robot.commands.autonomous.TwoPlaceTopBlue;
import frc.robot.commands.autonomous.TwoPlaceTopRed;
import frc.robot.commands.autonomous.balanceauto;
import frc.robot.commands.claw.AutoResetClawPosition;
import frc.robot.commands.claw.ResetClawPosition;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.AutoTurnToSubstation;
import frc.robot.commands.drivetrain.BalanceDrive;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.ChangeMaxSpeed;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.drivetrain.EstimatePose;
import frc.robot.commands.drivetrain.SprintDrive;
import frc.robot.commands.drivetrain.changeNeutralMode;
import frc.robot.commands.elevator.AutoResetElevatorPosition;
import frc.robot.commands.elevator.ResetElevatorPosition;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.limelight.MoveTo2DAprilTags;
import frc.robot.commands.limelight.MoveToRetroreflectiveTape;
import frc.robot.commands.limelight.read3DAprilTags;
import frc.robot.commands.limelight.readRetroreflectiveTape;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
  private final PowerDistribution pdp = new PowerDistribution(30, ModuleType.kRev);

  private final LED m_LED = new LED(pdp);
  private final Claw m_claw = new Claw();
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LimeLight m_limelight = new LimeLight();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake(pdp);

  SendableChooser<Command> m_chooser = new SendableChooser<>();
  private final balanceauto m_balanceauto = new balanceauto(m_drivetrain);
  private final PlaceConeAuto m_PlaceConeAuto = new PlaceConeAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceCubeAuto m_PlaceCubeAuto = new PlaceCubeAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeBalanceAuto m_PlaceConeBalanceAuto = new PlaceConeBalanceAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeRightBalanceAuto m_PlaceConeRightBalanceAuto = new PlaceConeRightBalanceAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeLeftBalanceAuto m_PlaceConeLeftBalanceAuto = new PlaceConeLeftBalanceAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceCubeBalanceAuto m_PlaceCubeBalanceAuto = new PlaceCubeBalanceAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeMobilityAuto m_PlaceConeMobilityAuto = new PlaceConeMobilityAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceCubeMobilityAuto m_PlaceCubeMobilityAuto = new PlaceCubeMobilityAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final TwoPlaceParkTopBlue m_TwoPlaceParkTopBlue = new TwoPlaceParkTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceParkTopRed m_TwoPlaceParkTopRed = new TwoPlaceParkTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceTopBlue m_TwoPlaceTopBlue = new TwoPlaceTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceTopRed m_TwoPlaceTopRed = new TwoPlaceTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final TwoPlaceBumpBlue m_TwoPlaceBumpBlue = new TwoPlaceBumpBlue(m_drivetrain, m_elevator, m_claw, m_intake);
  private final TwoPlaceBumpRed m_TwoPlaceBumpRed = new TwoPlaceBumpRed(m_drivetrain, m_elevator, m_claw, m_intake);
  private final ThreePlaceTopBlue m_ThreePlaceTopBlue = new ThreePlaceTopBlue(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final ThreePlaceTopRed m_ThreePlaceTopRed = new ThreePlaceTopRed(m_drivetrain, m_intake, m_elevator, m_claw, m_limelight);
  private final PlaceConeGrabBumpBlueAuto m_PlaceConeGrabBumpBlueAuto = new PlaceConeGrabBumpBlueAuto(m_drivetrain, m_elevator, m_claw, m_intake);
  private final PlaceConeGrabBumpAutoRed m_PlaceConeGrabBumpAuto = new PlaceConeGrabBumpAutoRed(m_drivetrain, m_elevator, m_claw, m_intake);

  private boolean coneMode = true;

  private boolean isTuckPos = true;
  private boolean isPickUpPos = false;
  private boolean isPlaceMidPos = false;
  private boolean isPlaceHighPos = false;
  private boolean isPrePlacePos = false;
  private boolean isDoubleSubstationPos = false;

  private Timer timeSinceLastMove;

  public RobotContainer() {
    // Configure the trigger bindings
    timeSinceLastMove = new Timer();
    timeSinceLastMove.start();

    // testBindings();
    configureBindings();

    SmartDashboard.putBoolean("Cone Mode", coneMode);
  
    m_drivetrain.setDefaultCommand(new DriveContinous(m_drivetrain));

    m_chooser.setDefaultOption("Two Place And Balance Blue", m_TwoPlaceParkTopBlue);
    m_chooser.addOption("Two Place And Balance Red", m_TwoPlaceParkTopRed);
    m_chooser.addOption("Two Place Blue", m_TwoPlaceTopBlue);
    m_chooser.addOption("Two Place Red", m_TwoPlaceTopRed);
    m_chooser.addOption("Two Place Bump Blue", m_TwoPlaceBumpBlue);
    m_chooser.addOption("Two Place Bump Red", m_TwoPlaceBumpRed);
    m_chooser.addOption("Place cone and mobility", m_PlaceConeMobilityAuto);
    m_chooser.addOption("Place cube and mobility", m_PlaceCubeMobilityAuto);
    m_chooser.addOption("Place cone and balance", m_PlaceConeBalanceAuto);
    m_chooser.addOption("Place cone LEFT and balance", m_PlaceConeLeftBalanceAuto);
    m_chooser.addOption("Place cone RIGHT and balance", m_PlaceConeRightBalanceAuto);
    m_chooser.addOption("Place cube and balance", m_PlaceCubeBalanceAuto);
    m_chooser.addOption("Place cone only", m_PlaceConeAuto);
    m_chooser.addOption("Place cube only", m_PlaceCubeAuto);
    m_chooser.addOption("Balance only", m_balanceauto);
    m_chooser.addOption("Place Three Blue", m_ThreePlaceTopBlue);
    m_chooser.addOption("Place Three Red", m_ThreePlaceTopRed);
    m_chooser.addOption("Place cone grab bump Blue", m_PlaceConeGrabBumpBlueAuto);
    m_chooser.addOption("Place cone grab bump Red", m_PlaceConeGrabBumpAuto); 
    SmartDashboard.putData(m_chooser);

    m_LED.setYellow();
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

  public void changeDriveNeutralMode(NeutralMode mode) {
    m_drivetrain.setNeutralMode(mode);
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

  private void testBindings() {
    new Trigger(m_controller::getAButton)
      // .onTrue(new InstantCommand(m_LED::turnOnRed));
      .whileTrue(new InstantCommand(this::testEncoder));

    // new Trigger(m_controller::getBButton)
    //   .onTrue(new InstantCommand(m_LED::turnOnBlue));

    // new Trigger(m_controller::getXButton)
    //   .onTrue(new InstantCommand(m_LED::turnOnGreen));

    // new Trigger(m_controller::getYButton)
    //   .onTrue(new InstantCommand(m_LED::turnOff));
  }

  private void configureBindings() {

    new Trigger(this::isConeMode)
      .and(this::isPieceNotIn)
        .whileTrue(new InstantCommand(m_LED::setYellow));

    new Trigger(this::isConeMode)
      .and(this::isPieceIn)
        .whileTrue(new FlashYellow(m_LED));

    new Trigger(this::isCubeMode)
      .and(this::isPieceNotIn)
        .whileTrue(new InstantCommand(m_LED::setPurple));

    new Trigger(this::isCubeMode)
      .and(this::isPieceIn)
        .whileTrue(new FlashPurple(m_LED));

    new Trigger(this::isPieceIn)
      .and(this::getIsPickUpPos)
        .onTrue(new SequentialCommandGroup(
          new WaitCommand(0.15),
          new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos),
          new InstantCommand(this::setToTuckPos)
        ));
        // .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        // .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(this::isPieceIn)
      .and(this::getIsDoubleSubstationPos)
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos));
    
    // new Trigger(this::isPieceIn)
    //   .and(this::getIsPickUpPos)
    //   .and(this::isConeMode)
    //   .onTrue(new InstantCommand(m_drivetrain::resetPidgeyToSubStation));

    new Trigger(zeroRobotControl)
      .onTrue(new InstantCommand(m_drivetrain::resetPidgey))
      .onTrue(new ZeroElevatorAndClaw(m_elevator, m_claw));
      
    new Trigger(intakeControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new ParallelCommandGroup(
          new RunTempIntake(m_intake, kIntakeSpeed),
          // new InstantCommand(this::setToPickUpPos),
          new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpConePos, armConePickupPos)
        ));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsPlaceHighPos)
        .whileTrue(new RunTempIntake(m_intake, -kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new TuckFromPlace(m_elevator, m_claw, m_intake, m_drivetrain));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsNotPickUpPos)
      .and(this::getIsNotPlaceHighPos)
        .whileTrue(new RunTempIntake(m_intake, -kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsPickUpPos)
        .whileTrue(new RunTempIntake(m_intake, -kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(intakeControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new ParallelCommandGroup(
          new RunTempIntake(m_intake, -kIntakeSpeed),
          // new InstantCommand(this::setToPickUpPos),
          new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpCubePos, armCubePickupPos)
        ));
  
    new Trigger(outtakeControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPickUpPos)
        .whileTrue(new RunTempIntake(m_intake, 0.5))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(outtakeControl)
      .and(this::isCubeMode)
      .and(this::getIsPickUpPos)
        .whileTrue(new RunTempIntake(m_intake, 0.5))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceConeHighTeleOp(m_elevator, m_claw, m_drivetrain));

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceConeHighFromPrePlaceTeleOp(m_elevator, m_claw, m_drivetrain),
          new InstantCommand(this::setToPlaceHighPos)
        ));
        // .onTrue(new PlaceConeHighFromPrePlaceTeleOp(m_elevator, m_claw, m_drivetrain));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeHighTeleOp(m_elevator, m_claw, m_intake, m_drivetrain),
          new InstantCommand(this::setToTuckPos)
        ));
        // .onTrue(new PlaceCubeHighTeleOp(m_elevator, m_claw, m_intake, m_drivetrain));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeHighFromPrePlaceTeleOp(m_elevator, m_claw, m_intake, m_drivetrain),
          new InstantCommand(this::setToTuckPos)
        ));
        // .onTrue(new PlaceCubeHighFromPrePlaceTeleOp(m_elevator, m_claw, m_intake, m_drivetrain));

    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new PlaceConeMidTeleOp(m_elevator, m_claw, m_drivetrain)); 
    
    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceMidPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new SetClawPosition(m_claw, armMidPlaceConePos)); 

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeMidTeleOp(m_elevator, m_claw, m_intake, m_drivetrain),
          new InstantCommand(this::setToTuckPos)
        ));
        // .onTrue(new PlaceCubeMidTeleOp(m_elevator, m_claw, m_intake, m_drivetrain)); 

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new RunTempIntake(m_intake, 0.5).withTimeout(0.25),
          new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos),
          new InstantCommand(this::setToTuckPos)
        ));
        // .onTrue(new SequentialCommandGroup(
        //   new RunTempIntake(m_intake, 0.5).withTimeout(0.25),
        //   new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos)
        // )); 

    // new Trigger(autoFloorConePickUpControl)
    //     .onTrue(new InstantCommand(this::setToPickUpPos))
    //     .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpConePos, armConePickupPos));

    new Trigger(autoSubstationConePickUpControl)
      .and(this::isConeMode)
        .onTrue(new ParallelCommandGroup(
          new InstantCommand(this::setToPickUpPos),
          new RunTempIntake(m_intake, kIntakeSpeed),
          new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorSubstationPos, armSubstationPos)
        ))
        .whileTrue(new AutoTurnToSubstation(m_drivetrain));

    new Trigger(autoDoubleSubstationConePickUpControl)
      .and(this::isConeMode)
          .onTrue(new ParallelCommandGroup(
            new InstantCommand(this::setToDoubleSubstationPos),
            new RunTempIntake(m_intake, kIntakeSpeed),
            new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorDoubleSubstationPos, armDoubleSubstationPos)
          ))
          .whileTrue(new AutoTurn(m_drivetrain, 0));

    new Trigger(autoDoubleSubstationConePickUpControl)
      .and(this::isCubeMode)
          .whileTrue(new RunTempIntake(m_intake, 1.0));
  
    // new Trigger(autoPickUpControl)
    //   .and(this::isCubeMode)
    //     .onTrue(new InstantCommand(this::setToPickUpPos))
    //     .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorPickUpCubePos, armCubePickupPos));

    new Trigger(autoTuckControl)
      .and(this::getIsPickUpPos)
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndClaw(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(autoTuckControl)
      .and(this::getIsNotPickUpPos)
        .onTrue(new InstantCommand(this::setToTuckPos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorTuckPos, armTuckPos));

    new Trigger(this::getIsTuckPos)
      .and(this::isConeMode)
        .onTrue(new RunTempIntake(m_intake, 0.2));

    new Trigger(this::getIsTuckPos)
      .and(this::isCubeMode)
        .onTrue(new RunTempIntake(m_intake, -0.15));

    new Trigger(lockWheelsControl)
      .onTrue(new changeNeutralMode(m_drivetrain, NeutralMode.Brake))
      .whileTrue(new BalanceDrive(m_drivetrain))
      .onFalse(new changeNeutralMode(m_drivetrain, NeutralMode.Coast));
      // .whileTrue(new AutoBalance(m_drivetrain));

    new Trigger(changeModeControl)
      .whileTrue(new RunTempIntake(m_intake, 0.0))
      .onTrue(new InstantCommand(this::changeMode));
    
    new Trigger(prePlaceControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPrePlacePos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorConeMidPos, armPrePlacePos));

    new Trigger(prePlaceControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPrePlacePos))
        .onTrue(new MoveElevatorAndClawFast(m_elevator, m_claw, elevatorCubeMidPos, armMidPlaceCubePos));

    new Trigger(autoLineUpControl)
      .and(this::isConeMode)
        // .whileTrue(new SequentialCommandGroup(
        //   new AutoTurn(m_drivetrain, 180),
        //   new MoveToRetroreflectiveTape(m_limelight, m_drivetrain)
        // ));
        .whileTrue(new MoveToRetroreflectiveTape(m_limelight, m_drivetrain, 180));
        // .whileTrue(new MoveToRetroreflectiveTape(m_limelight, m_drivetrain));
    
    new Trigger(autoLineUpControl)
      .and(this::isCubeMode)
        .whileTrue(new SequentialCommandGroup(
          new AutoTurn(m_drivetrain, 180),
          new MoveTo2DAprilTags(m_limelight, m_drivetrain, 180)
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

  public boolean isPieceIn() {
    if (isTuckPos && timeSinceLastMove.get() > 0.5) {
      if (isConeMode() && m_intake.getCurrentDraw() > 2){
        return true;
      } else if (isCubeMode() && m_intake.getCurrentDraw() > 1.2) {
        return true;
      } else {
        return false;
      }
    } else if (m_intake.getCurrentDraw() > 25 && timeSinceLastMove.get() > 0.5) {
      return true;
    } else {
      return false;
    }
  }

  public boolean isPieceNotIn() {
    return !isPieceIn();
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
    // System.out.println("Tuck");
    isTuckPos = true;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPickUpPos() {
    // System.out.println("Pick Up");
    isTuckPos = false;
    isPickUpPos = true;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPlaceMidPos() {
    // System.out.println("Mid");
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = true;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPlaceHighPos() {
    // System.out.println("High");
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = true;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPrePlacePos() {
    // System.out.println("High");
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = true;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToDoubleSubstationPos() {
    // System.out.println("High");
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = true;
    timeSinceLastMove.reset();
  }

  public void putAllianceColor() {
    System.out.println(DriverStation.getAlliance().name());
  }

  public boolean getIsPlacePos() {
    return isPlaceHighPos || isPlaceMidPos;
  }

  public boolean getIsDoubleSubstationPos() {
    return isDoubleSubstationPos;
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

  public boolean getIsPrePlacePos() {
    return isPrePlacePos;
  }

  public boolean getIsNotPrePlacePos() {
    return !isPrePlacePos;
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
    // return m_drivetrain.followPathCommand(true, "TestPath");
    // return new PlaceCubeHighAuto(m_elevator, m_claw, m_intake);
  }
}
