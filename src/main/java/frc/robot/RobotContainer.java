// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;

// import org.ejml.dense.block.decomposition.chol.InnerCholesky_DDRB;

import frc.robot.commands.AutoPlaceConeHigh;
import frc.robot.commands.AutoResetElevatorAndClaw;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.auto.DriveForward;
import frc.robot.commands.auto.DriveUp;
import frc.robot.commands.auto.TestPath;
import frc.robot.commands.auto.balanceauto;
import frc.robot.commands.claw.ResetClawPosition;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.SetClawPositionWaitForFinish;
import frc.robot.commands.claw.ToggleGrip;
import frc.robot.commands.claw.detectSensor;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.ChangeDrivePID;
import frc.robot.commands.drivetrain.DriveContinous;
import frc.robot.commands.elevator.RunElevator;
import frc.robot.commands.intake.RunIntake;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.intake.SpinIntakeParallel;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.ClawPneumatics;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;
import frc.robot.subsystems.tempIntake;
import edu.wpi.first.wpilibj.XboxController;
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
  private final XboxController m_controller = new XboxController(0);
  private final Claw m_claw = new Claw();
  private final ClawPneumatics m_ClawPneumatics = new ClawPneumatics();

  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final LimeLight m_limelight = new LimeLight();
  //private final motionMagicMotor m_motionMagicMotor = new motionMagicMotor();
  private final Elevator m_elevator = new Elevator();
  private final Intake m_intake = new Intake();
  private final tempIntake m_tempIntake = new tempIntake();

  private final DriveUp m_DriveUp = new DriveUp(m_drivetrain);
  private final DriveForward m_DriveForward = new DriveForward(m_drivetrain);
  private final TestPath m_TestPath = new TestPath(m_drivetrain);
  private final balanceauto m_balanceauto = new balanceauto(m_drivetrain);

  private boolean isFast = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  
    m_drivetrain.setDefaultCommand(new DriveContinous(m_drivetrain, m_controller));
  }

  public void testEncoder() {
    m_drivetrain.printEncoders();
    // m_drivetrain.drive(0, 0, 0, false);
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

    // new Trigger(this::getDpadUp)
    //   .onTrue(new PrintCommand("Up"));
    // new Trigger(this::getDpadRight)
    //   .onTrue(new PrintCommand("Right"));
    // new Trigger(this::getDpadDown)
    //   .onTrue(new PrintCommand("Down"));
    // new Trigger(this::getDpadLeft)
    //   .onTrue(new PrintCommand("Left"));

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

    // new Trigger(m_controller::getBButton).whileTrue(new setMotorToPosition(m_motionMagicMotor, posOne));
    // new Trigger(m_controller::getAButton).whileTrue(new setMotorToPosition(m_motionMagicMotor, posTwo));

    //new Trigger(m_controller::getAButton).whileTrue(new moveElevator(m_elevator,elevator20Inches));
    //new Trigger(m_controller::getBButton).whileTrue(new moveElevator(m_elevator,elevatorStartPos));

    // new Trigger(m_controller::getAButton)
    //     .onTrue(new SpinIntake(m_intake, -1.0))
    //     .onFalse(new StopIntake(m_intake));
    // new Trigger(m_controller::getBButton)
    //     .onTrue(new SpinIntake(m_intake, 1.0))
    //     .onFalse(new StopIntake(m_intake));
    // new Trigger(m_controller::getRightBumper)
    //   .onTrue(new SpinIntakeParallel(m_intake, -1.0))
    //   .onFalse(new StopIntake(m_intake));
    // new Trigger(m_controller::getLeftBumper)
    //   .onTrue(new SpinIntakeParallel(m_intake, 1.0))
    //   .onFalse(new StopIntake(m_intake));

    //Intake
    new Trigger(m_controller::getRightBumper)
      // .whileTrue(new RunTempIntake(m_tempIntake, kIntakeSpeed));
      .whileTrue(new RunIntake(m_intake, kIntakeSpeed))
      // .whileTrue(new ParallelCommandGroup(
      //   new SetIntakePosition(m_intake, true),
      //   new SpinIntake(m_intake, kIntakeSpeed)
      // ))
      .onFalse(new ParallelDeadlineGroup(
        new WaitCommand(2),
        new SetIntakePosition(m_intake, false),
        new SpinIntakeParallel(m_intake, kIntakeSpeed)
      ));

    new Trigger(m_controller::getYButton)
      // .whileTrue(new RunTempIntake(m_tempIntake, -kIntakeSpeed));
      // .whileTrue(new detectSensor(m_claw));
      .whileTrue(new SpinIntakeParallel(m_intake, -kIntakeSpeed));
      // .whileTrue(new GetYaw(m_drivetrain));



    //Elevator
    new Trigger(this::getDpadRight)
      // .whileTrue(new PrintCommand("top"))
      // .whileTrue(new InstantCommand(m_elevator::runElevator));
      // .whileTrue(new MoveElevator(m_elevator, elevatorUpPos));
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      .onTrue(new AutoPlaceConeHigh(m_intake, m_elevator, m_claw, m_ClawPneumatics));

    new Trigger(this::getDpadLeft)
      .onTrue(new AutoResetElevatorAndClaw(m_elevator, m_claw, m_ClawPneumatics));
    //   .whileTrue(new PrintCommand("mid"))
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      // .whileTrue(new MoveElevator(m_elevator, elevatorStartPos));

    new Trigger(this::isFast)
      .and(m_controller::getStartButton)
        .onTrue(new PrintCommand("isFast"))
        .onTrue(new ChangeDrivePID(m_drivetrain, 0.01, 0.001, 0))
        .onFalse(new InstantCommand(this::changeSpeed));
      // .onFalse(new ChangeDrivePID(m_drivetrain, 1.0, 0.001, 0));
    //   .onTrue(new AutoPlaceConeHigh(m_intake, m_elevator, m_claw, m_ClawPneumatics));
    
    new Trigger(this::isSlow)
      .and(m_controller::getStartButton)
        .onTrue(new PrintCommand("isSlow"))
        .onTrue(new ChangeDrivePID(m_drivetrain, 1.0, 0.001, 0))
        .onFalse(new InstantCommand(this::changeSpeed));

    new Trigger(m_controller::getBackButton)
      // .whileTrue(new PrintCommand("Box wheels"))
      // .whileTrue(new InstantCommand(m_drivetrain::boxWheels));
      .whileTrue(new AutoBalance(m_drivetrain));

      //.onTrue(new AutoResetElevatorAndClaw(m_elevator, m_claw, m_ClawPneumatics));
      // .whileTrue(new AutoResetElevatorAndClaw(m_elevator, m_claw, m_ClawPneumatics));
      // .whileTrue(new SetClawPosition(m_claw, armDownPos));

     //new Trigger(m_controller::getAButton)
    //   .onTrue(new PrintCommand("start"))
    //   .onTrue(new MoveElevator(m_elevator, elevatorStartPos));

    new Trigger(this::getDpadUp)
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      .whileTrue(new RunElevator(m_elevator, 1.0)) // change
      .onFalse(new RunElevator(m_elevator, 0.0));
      // .whileTrue(new MoveElevatorManual(m_elevator, elevatorTicksPerInches * 0.4));
    
    new Trigger(this::getDpadDown)
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      // .whileTrue(new InstantCommand(m_elevator::printPosition))
      .whileTrue(new RunElevator(m_elevator, -1.0)) // change
      .onFalse(new RunElevator(m_elevator, 0.0));
      // .whileTrue(new MoveElevatorManual(m_elevator, elevatorTicksPerInches * -0.4));


    //Claw
    new Trigger(this::getNotLeftBumper)
      .and(new Trigger(m_controller::getXButton))
        // .whileTrue(new InstantCommand(m_claw::printPosition))
        .whileTrue(new InstantCommand(m_claw::runMotorForward))
        .onFalse(new InstantCommand(m_claw::stopMotor));
        // .whileTrue(new MoveClaw(m_claw, armTicksPerDegree * 0.8));
        // .whileTrue(new SetClawPosition(m_claw, armDownPos));

    new Trigger(this::getNotLeftBumper)
      .and(new Trigger(m_controller::getBButton))
        // .whileTrue(new InstantCommand(m_claw::printPosition))
        .whileTrue(new InstantCommand(m_claw::runMotorBackward))
        .onFalse(new InstantCommand(m_claw::stopMotor));
        // .whileTrue(new MoveClaw(m_claw, armTicksPerDegree * -0.8));
        // .whileTrue(new SetClawPosition(m_claw, armPlacePos));

    new Trigger(this::getNotLeftBumper)
      .and(new Trigger(m_controller::getAButton))
        .toggleOnTrue(new ToggleGrip(m_ClawPneumatics));
        // .whileTrue(new InstantCommand(m_claw::closeClaw, m_claw))
        // .whileFalse(new InstantCommand(m_claw::openClaw, m_claw));

    new Trigger(m_controller::getLeftBumper)
      .and(new Trigger(m_controller::getXButton))
        // .onTrue(new SequentialCommandGroup(
        //   new SetClawPositionWaitForFinish(m_claw, armPlacePos),
        //   new PrintCommand("done")
        // ));
        .whileTrue(new SetClawPosition(m_claw, armPlacePos));
        // .whileTrue(new MoveClaw(m_claw, armTicksPerDegree * 0.8));

    new Trigger(m_controller::getLeftBumper)
        .and(new Trigger(m_controller::getBButton))
          // .onTrue(new SequentialCommandGroup(
          //   new SetClawPositionWaitForFinish(m_claw, armDownPos),
          //   new PrintCommand("done")
          // ));
          .whileTrue(new SetClawPosition(m_claw, armDownPos));
          // .whileTrue(new MoveClaw(m_claw, armTicksPerDegree * -0.8));

    new Trigger(m_controller::getLeftBumper)
      .and(new Trigger(m_controller::getAButton))
        .whileTrue(new ResetClawPosition(m_claw));
  }

  public void changeSpeed() {
    isFast = !isFast;
  }

  public boolean isFast() {
    return isFast;
  }

  public boolean isSlow() {
    return !isFast;
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

  public boolean getDpadUp() {
    if (m_controller.getPOV() == 0) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getDpadRight() {
    if (m_controller.getPOV() == 90) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getDpadDown() {
    if (m_controller.getPOV() == 180) {
      return true;
    } else {
      return false;
    }
  }

  public boolean getDpadLeft() {
    if (m_controller.getPOV() == 270) {
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
    // An example command will be run in autonomous
    return m_balanceauto;
    // return Autos.exampleAuto();
  }
}
