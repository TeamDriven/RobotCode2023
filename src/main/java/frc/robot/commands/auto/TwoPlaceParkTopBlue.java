// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.MotionMagicConstants.*;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveElevatorAndClaw;
import frc.robot.commands.AutoMoveElevatorAndClawFast;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.claw.MoveClaw;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.SetClawPositionWaitForFinish;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.SetPidgey;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.Claw;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class TwoPlaceParkTopBlue extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public TwoPlaceParkTopBlue(Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw) {
    List<PathPlannerTrajectory> pathList = PathPlanner.loadPathGroup(
      "TwoPlaceParkTopBlue", 
      new PathConstraints(3, 3), 
      new PathConstraints(3, 3), 
      new PathConstraints(2, 3),
      new PathConstraints(3, 4)
    );
    addCommands(
      // new AutoMoveElevatorAndClaw(elevator, claw, elevatorUpPosAuto, armPlacePosAuto),
      new MoveElevator(elevator, elevatorConeUpPos),
      new WaitCommand(0.6),
      new SetClawPosition(claw, armHighPlaceConePos),
      new WaitCommand(0.7),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25),
        new RunTempIntake(intake, -.5)
      ),
      // new AutoMoveElevatorAndClawFast(elevator, claw, elevatorTuckPos, armTuckPos),
      new SetClawPosition(claw, armTuckPos),
      new WaitCommand(0.05),
      new MoveElevator(elevator, elevatorTuckPos),
      drivetrain.followPathCommand(true, pathList.get(0)),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(false, pathList.get(1)),
        new AutoMoveElevatorAndClawFast(elevator, claw, elevatorPickUpCubePos, armTicksPerDegree * 80),
        new RunTempIntake(intake, -1)
        // new ParallelDeadlineGroup(
        //   new WaitCommand(1.5), 
        //   new RunTempIntake(intake, -1)
        // )
      ),
      new ParallelCommandGroup(
        new AutoMoveElevatorAndClaw(elevator, claw, elevatorTuckPos, armTuckPos),
        drivetrain.followPathCommand(false, pathList.get(2))
      ),
      new MoveElevator(elevator, elevatorTicksPerInches * 40),
      new WaitCommand(0.3),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new RunTempIntake(intake, 1)
      ),
      new MoveElevator(elevator, elevatorTuckPos),
      drivetrain.followPathCommand(false, pathList.get(3)),
      new ParallelDeadlineGroup(
        new WaitCommand(1.5), 
        new Drive(drivetrain, -2.5, 0, 0, true)
      ),
      new AutoBalance(drivetrain),
      new InstantCommand(drivetrain::boxWheels)
    );
  }
}