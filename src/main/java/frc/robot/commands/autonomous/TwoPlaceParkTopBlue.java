// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.MotionMagicConstants.*;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Claw;
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
      new PlaceConeHighAuto(elevator, claw, intake),
      drivetrain.followPathCommand(true, pathList.get(0)),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(false, pathList.get(1)),
        new MoveElevatorAndClawFast(elevator, claw, elevatorPickUpCubePos, armTicksPerDegree * 80),
        new RunTempIntake(intake, -1)
      ),
      new ParallelCommandGroup(
        new MoveElevatorAndClaw(elevator, claw, elevatorTuckPos, armTuckPos),
        drivetrain.followPathCommand(false, pathList.get(2))
      ),
      new PlaceCubeHighAuto(elevator, claw, intake),
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