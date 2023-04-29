// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.MotionMagicConstants.*;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.automation.MoveElevatorAndClaw;
import frc.robot.commands.automation.MoveElevatorAndClawFast;
import frc.robot.commands.automation.PlaceConeHighAuto;
import frc.robot.commands.automation.PlaceCubeHighAuto;
import frc.robot.commands.automation.ZeroElevatorAndClaw;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.MoveToLimelight;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.limelight.MoveTo2DAprilTags;
import frc.robot.commands.limelight.read2DAprilTagSnapshot;
import frc.robot.commands.limelight.read2DAprilTags;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

public final class TwoPlaceTopRed extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public TwoPlaceTopRed(Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw, LimeLight limeLight) {
    List<PathPlannerTrajectory> pathList = PathPlanner.loadPathGroup(
      "TwoPlaceParkTopRed", 
      new PathConstraints(3, 3), 
      new PathConstraints(3, 3),
      new PathConstraints(3, 3)
    );

    PIDController pTheta1 = new PIDController(2.5, 0, 0);
    PIDController pTheta2 = new PIDController(1.5, 0, 0);
    // PIDController pTheta3 = new PIDController(2.5, 0, 0);

    addCommands(
      new ZeroElevatorAndClaw(elevator, claw),
      new SetClawPosition(claw, armTuckPos),
      new WaitCommand(0.1),
      new PlaceConeHighAuto(elevator, claw, intake, drivetrain),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(true, pathList.get(0), pTheta1),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
            new MoveElevatorAndClawFast(elevator, claw, elevatorPickUpCubePos, armCubePickupPos),
            new RunTempIntake(intake, -1)
          )
        )
      ),
      new ParallelCommandGroup(
        new MoveElevatorAndClaw(elevator, claw, elevatorTuckPos, armTuckPos),
        drivetrain.followPathCommand(false, pathList.get(1), pTheta2)
      ),
      // new RunTempIntake(intake, 0.4).withTimeout(0.2)
      new ParallelDeadlineGroup(
        new PlaceCubeHighAuto(elevator, claw, intake, drivetrain),
        new Drive(drivetrain, 0, 0, 0, true)
      )
    );
  }
}