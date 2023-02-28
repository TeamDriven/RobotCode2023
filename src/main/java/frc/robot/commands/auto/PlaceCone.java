// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.HashMap;

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
import frc.robot.commands.AutoPlaceConeHigh;
import frc.robot.commands.AutoPlaceConeHighAuto;
import frc.robot.commands.AutoResetElevatorAndClaw;
import frc.robot.commands.RunTempIntake;
import frc.robot.subsystems.Claw;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class PlaceCone extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public PlaceCone(Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw) {
    addCommands(
      new AutoPlaceConeHighAuto(intake, elevator, claw),
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new RunTempIntake(intake, -.5)
      ),
      new AutoResetElevatorAndClaw(elevator, claw),
      drivetrain.followPathCommand(true, "Taxi")
    );
  }

}