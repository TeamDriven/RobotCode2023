// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoMoveElevatorAndClawFast;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.Claw;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class TestPath extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public TestPath(final Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw) {
    addCommands(
      // new AutoMoveElevatorAndClaw(elevator, claw, elevatorCubeUpPos, armTuckPos),
      new SetClawPosition(claw, armTuckPos),
      new MoveElevator(elevator, elevatorCubeMidPos),
      new WaitCommand(0.1),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new RunTempIntake(intake, 0.75)
      ),
      new MoveElevator(elevator, elevatorTuckPos)
    );
  }

}