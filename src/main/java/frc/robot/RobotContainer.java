// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
import static frc.robot.Controls.*;
import static frc.robot.SubsystemInstances.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import frc.robot.commands.FlashPurple;
import frc.robot.commands.FlashYellow;
import frc.robot.commands.RunIntake;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.automation.MoveElevatorAndArm;
import frc.robot.commands.automation.MoveElevatorAndArmFast;
import frc.robot.commands.automation.PlaceConeHighFromPrePlaceTeleOp;
import frc.robot.commands.automation.PlaceConeHighTeleOp;
import frc.robot.commands.automation.PlaceConeMidTeleOp;
import frc.robot.commands.automation.PlaceCubeHighFromPrePlaceTeleOp;
import frc.robot.commands.automation.PlaceCubeHighTeleOp;
import frc.robot.commands.automation.PlaceCubeMidTeleOp;
import frc.robot.commands.automation.TuckFromPlace;
import frc.robot.commands.automation.ZeroElevatorAndArm;
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
import frc.robot.commands.autonomous.BalanceAuto;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.AutoTurnToSubstation;
import frc.robot.commands.drivetrain.BalanceDrive;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.drivetrain.ChangeNeutralMode;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.limelight.MoveTo2DAprilTags;
import frc.robot.commands.limelight.MoveToRetroreflectiveTape;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  SendableChooser<Command> m_chooser = new SendableChooser<>();

  private boolean coneMode = true;

  private boolean isTuckPos = true;
  private boolean isPickUpPos = false;
  private boolean isPlaceMidPos = false;
  private boolean isPlaceHighPos = false;
  private boolean isPrePlacePos = false;
  private boolean isDoubleSubstationPos = false;

  private Timer timeSinceLastMove;

  public RobotContainer() {
    timeSinceLastMove = new Timer();
    timeSinceLastMove.start();

    // testBindings();
    configureBindings();

    SmartDashboard.putBoolean("Cone Mode", coneMode);
  
    drivetrain.setDefaultCommand(new DriveContinous());

    m_chooser.setDefaultOption("Two Place And Balance Blue", new TwoPlaceParkTopBlue());
    m_chooser.addOption("Two Place And Balance Red", new TwoPlaceParkTopRed());
    m_chooser.addOption("Two Place Blue", new TwoPlaceTopBlue());
    m_chooser.addOption("Two Place Red", new TwoPlaceTopRed());
    m_chooser.addOption("Two Place Bump Blue", new TwoPlaceBumpBlue());
    m_chooser.addOption("Two Place Bump Red", new TwoPlaceBumpRed());
    m_chooser.addOption("Place cone and mobility", new PlaceConeMobilityAuto());
    m_chooser.addOption("Place cube and mobility", new PlaceCubeMobilityAuto());
    m_chooser.addOption("Place cone and balance", new PlaceConeBalanceAuto());
    m_chooser.addOption("Place cone LEFT and balance", new PlaceConeLeftBalanceAuto());
    m_chooser.addOption("Place cone RIGHT and balance", new PlaceConeRightBalanceAuto());
    m_chooser.addOption("Place cube and balance", new PlaceCubeBalanceAuto());
    m_chooser.addOption("Place cone only", new PlaceConeAuto());
    m_chooser.addOption("Place cube only", new PlaceCubeAuto());
    m_chooser.addOption("Balance only", new BalanceAuto());
    m_chooser.addOption("Place Three Blue", new ThreePlaceTopBlue());
    m_chooser.addOption("Place Three Red", new ThreePlaceTopRed());
    m_chooser.addOption("Place cone grab bump Blue", new PlaceConeGrabBumpBlueAuto());
    m_chooser.addOption("Place cone grab bump Red", new PlaceConeGrabBumpAutoRed()); 
    SmartDashboard.putData(m_chooser);

    LED.setYellow();
  }

  public void testBindings() {
    new Trigger(m_controller::getAButton)
      // .onTrue(new InstantCommand(m_LED::turnOnRed));
      .whileTrue(new InstantCommand(drivetrain::printEncoders));

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
        .whileTrue(new InstantCommand(LED::setYellow));

    new Trigger(this::isConeMode)
      .and(this::isPieceIn)
        .whileTrue(new FlashYellow());

    new Trigger(this::isCubeMode)
      .and(this::isPieceNotIn)
        .whileTrue(new InstantCommand(LED::setPurple));

    new Trigger(this::isCubeMode)
      .and(this::isPieceIn)
        .whileTrue(new FlashPurple());

    new Trigger(this::isPieceIn)
      .and(this::getIsPickUpPos)
        .onTrue(new SequentialCommandGroup(
          new WaitCommand(0.15),
          new MoveElevatorAndArm(elevatorTuckPos, armTuckPos),
          new InstantCommand(this::setToTuckPos)
        ));

    new Trigger(this::isPieceIn)
      .and(this::getIsDoubleSubstationPos)
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArmFast(elevatorTuckPos, armTuckPos));
    
    new Trigger(zeroRobotControl)
      .onTrue(new InstantCommand(drivetrain::resetPidgey))
      .onTrue(new ZeroElevatorAndArm());
      
    new Trigger(intakeControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new ParallelCommandGroup(
          new RunIntake(kIntakeSpeed),
          new MoveElevatorAndArmFast(elevatorPickUpConePos, armConePickupPos)
        ));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsPlaceHighPos)
        .whileTrue(new RunIntake(-kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new TuckFromPlace());

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsNotPickUpPos)
      .and(this::getIsNotPlaceHighPos)
        .whileTrue(new RunIntake(-kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArmFast(elevatorTuckPos, armTuckPos));

    new Trigger(outtakeControl)
      .and(this::isConeMode)
      .and(this::getIsPickUpPos)
        .whileTrue(new RunIntake(-kIntakeSpeed))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArm(elevatorTuckPos, armTuckPos));

    new Trigger(intakeControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPickUpPos))
        .onTrue(new ParallelCommandGroup(
          new RunIntake(-kIntakeSpeed),
          new MoveElevatorAndArmFast(elevatorPickUpCubePos, armCubePickupPos)
        ));
  
    new Trigger(outtakeControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPickUpPos)
        .whileTrue(new RunIntake(0.5))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArmFast(elevatorTuckPos, armTuckPos));

    new Trigger(outtakeControl)
      .and(this::isCubeMode)
      .and(this::getIsPickUpPos)
        .whileTrue(new RunIntake(0.5))
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArm(elevatorTuckPos, armTuckPos));

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceHighPos))
        .onTrue(new PlaceConeHighTeleOp());

    new Trigger(autoPlaceHighControl)
      .and(this::isConeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceConeHighFromPrePlaceTeleOp(),
          new InstantCommand(this::setToPlaceHighPos)
        ));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeHighTeleOp(),
          new InstantCommand(this::setToTuckPos)
        ));

    new Trigger(autoPlaceHighControl)
      .and(this::isCubeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeHighFromPrePlaceTeleOp(),
          new InstantCommand(this::setToTuckPos)
        ));

    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceMidPos))
        .onTrue(new PlaceConeMidTeleOp()); 
    
    new Trigger(autoPlaceMidControl)
      .and(this::isConeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new InstantCommand(this::setToPlaceMidPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new SetArmPosition(armMidPlaceConePos)); 

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
      .and(this::getIsNotPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new PlaceCubeMidTeleOp(),
          new InstantCommand(this::setToTuckPos)
        ));

    new Trigger(autoPlaceMidControl)
      .and(this::isCubeMode)
      .and(this::getIsPrePlacePos)
        .onTrue(new SequentialCommandGroup(
          new RunIntake(0.5).withTimeout(0.25),
          new MoveElevatorAndArmFast(elevatorTuckPos, armTuckPos),
          new InstantCommand(this::setToTuckPos)
        ));

    new Trigger(autoSubstationConePickUpControl)
      .and(this::isConeMode)
        .onTrue(new ParallelCommandGroup(
          new InstantCommand(this::setToPickUpPos),
          new RunIntake(kIntakeSpeed),
          new MoveElevatorAndArmFast(elevatorSubstationPos, armSubstationPos)
        ))
        .whileTrue(new AutoTurnToSubstation());

    new Trigger(autoDoubleSubstationConePickUpControl)
      .and(this::isConeMode)
          .onTrue(new ParallelCommandGroup(
            new InstantCommand(this::setToDoubleSubstationPos),
            new RunIntake(kIntakeSpeed),
            new MoveElevatorAndArmFast(elevatorDoubleSubstationPos, armDoubleSubstationPos)
          ))
          .whileTrue(new AutoTurn(0));

    new Trigger(autoDoubleSubstationConePickUpControl)
      .and(this::isCubeMode)
          .whileTrue(new RunIntake(1.0));

    new Trigger(autoTuckControl)
      .and(this::getIsPickUpPos)
        .onTrue(new InstantCommand(this::setToTuckPos).beforeStarting(new WaitCommand(0.2)))
        .onTrue(new MoveElevatorAndArm(elevatorTuckPos, armTuckPos));

    new Trigger(autoTuckControl)
      .and(this::getIsNotPickUpPos)
        .onTrue(new InstantCommand(this::setToTuckPos))
        .onTrue(new MoveElevatorAndArmFast(elevatorTuckPos, armTuckPos));

    new Trigger(this::getIsTuckPos)
      .and(this::isConeMode)
        .onTrue(new RunIntake(0.2));

    new Trigger(this::getIsTuckPos)
      .and(this::isCubeMode)
        .onTrue(new RunIntake(-0.15));

    new Trigger(lockWheelsControl)
      .onTrue(new ChangeNeutralMode(NeutralMode.Brake))
      .whileTrue(new BalanceDrive())
      .onFalse(new ChangeNeutralMode(NeutralMode.Coast));

    new Trigger(changeModeControl)
      .whileTrue(new RunIntake(0.0))
      .onTrue(new InstantCommand(this::changeMode));
    
    new Trigger(prePlaceControl)
      .and(this::isConeMode)
        .onTrue(new InstantCommand(this::setToPrePlacePos))
        .onTrue(new MoveElevatorAndArmFast(elevatorConeMidPos, armPrePlacePos));

    new Trigger(prePlaceControl)
      .and(this::isCubeMode)
        .onTrue(new InstantCommand(this::setToPrePlacePos))
        .onTrue(new MoveElevatorAndArmFast(elevatorCubeMidPos, armMidPlaceCubePos));

    new Trigger(autoLineUpControl)
      .and(this::isConeMode)
        .whileTrue(new MoveToRetroreflectiveTape(180));
    
    new Trigger(autoLineUpControl)
      .and(this::isCubeMode)
        .whileTrue(new SequentialCommandGroup(
          new AutoTurn(180),
          new MoveTo2DAprilTags(180)
        ));

    new Trigger(moveElevatorUpControl)
      .whileTrue(new RunElevator(0.5))
      .onFalse(new RunElevator(0.0));
    
    new Trigger(moveElevatorDownControl)
      .whileTrue(new RunElevator(-0.5))
      .onFalse(new RunElevator(0.0));

    new Trigger(moveArmUpControl)
      .whileTrue(new InstantCommand(arm::runMotorForward))
      .onFalse(new InstantCommand(arm::stopMotor));

      
    new Trigger(moveArmDownControl)
      .whileTrue(new InstantCommand(arm::runMotorBackward))
      .onFalse(new InstantCommand(arm::stopMotor));
 
  }

  public boolean isPieceIn() {
    if (isTuckPos && timeSinceLastMove.get() > 0.5) {
      if (isConeMode() && intake.getCurrentDraw() > 2){
        return true;
      } else if (isCubeMode() && intake.getCurrentDraw() > 1.2) {
        return true;
      } else {
        return false;
      }
    } else if (intake.getCurrentDraw() > 25 && timeSinceLastMove.get() > 0.5) {
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
    isTuckPos = true;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPickUpPos() {
    isTuckPos = false;
    isPickUpPos = true;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPlaceMidPos() {
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = true;
    isPlaceHighPos = false;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPlaceHighPos() {
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = true;
    isPrePlacePos = false;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToPrePlacePos() {
    isTuckPos = false;
    isPickUpPos = false;
    isPlaceMidPos = false;
    isPlaceHighPos = false;
    isPrePlacePos = true;
    isDoubleSubstationPos = false;
    timeSinceLastMove.reset();
  }

  public void setToDoubleSubstationPos() {
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
    return m_chooser.getSelected();
  }
}
