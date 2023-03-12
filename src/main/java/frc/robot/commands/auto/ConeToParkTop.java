// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import static frc.robot.Constants.MotionMagicConstants.*;

import java.util.HashMap;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveElevatorAndClaw;
import frc.robot.commands.AutoMoveElevatorAndClawFast;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.commands.drivetrain.SetPidgey;
import frc.robot.subsystems.Claw;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class ConeToParkTop extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public ConeToParkTop(Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw) {
    addCommands(
      new AutoMoveElevatorAndClaw(elevator, claw, elevatorUpPosAuto, armPlacePosAuto),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new RunTempIntake(intake, -.5)
      ),
      new AutoMoveElevatorAndClawFast(elevator, claw, elevatorTuckPos, armTuckPos),
      new ParallelCommandGroup(
        drivetrain.followPathCommand(true, "ConePickUpAndPark"),
        new AutoMoveElevatorAndClawFast(elevator, claw, elevatorPickUpCubePos, armCubePickupPos),
        new ParallelDeadlineGroup(
          new WaitCommand(4), 
          new RunTempIntake(intake, -1.0)
        )
      ),
      new ParallelCommandGroup(
        new AutoMoveElevatorAndClaw(elevator, claw, elevatorTuckPos, armTuckPos),
        new balanceauto(drivetrain)
      )
      // new Drive(drivetrain, 0, 0, 0, true),
      // new AutoMoveElevatorAndClaw(elevator, claw, elevatorCubeUpPos, armHighPlaceCubePos),
      // new ParallelDeadlineGroup(
      //   new WaitCommand(1),
      //   new RunTempIntake(intake, .5)
      // )
      // new SetPidgey(drivetrain, 180)
    );
  }
}