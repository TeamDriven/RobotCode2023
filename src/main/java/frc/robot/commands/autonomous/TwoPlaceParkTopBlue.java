// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.MotionMagicConstants.*;
import static frc.robot.SubsystemInstances.*;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.automation.MoveElevatorAndArm;
import frc.robot.commands.automation.MoveElevatorAndArmFast;
import frc.robot.commands.automation.PlaceConeHighAuto;
import frc.robot.commands.automation.ZeroElevatorAndArm;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.Drive;

public final class TwoPlaceParkTopBlue extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public TwoPlaceParkTopBlue() {
    List<PathPlannerTrajectory> pathList = PathPlanner.loadPathGroup(
      "TwoPlaceParkTopBlue", 
      new PathConstraints(3, 3), 
      new PathConstraints(3, 3),
      new PathConstraints(3, 3)
    );

    PIDController pTheta1 = new PIDController(2.5, 0, 0);
    PIDController pTheta2 = new PIDController(1.5, 0, 0);
    
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(14.75), //14.75
        new SequentialCommandGroup(
          new ZeroElevatorAndArm(),
          new SetArmPosition(armTuckPos),
          new WaitCommand(0.1),
          new PlaceConeHighAuto(),
          new ParallelDeadlineGroup(
            drivetrain.followPathCommand(true, pathList.get(0), pTheta1),
            new SequentialCommandGroup(
              new WaitCommand(1),
              new ParallelCommandGroup(
                new MoveElevatorAndArmFast(elevatorPickUpCubePos, armCubePickupPos),
                new RunIntake(-1)
              )
            )
          ),
          new ParallelCommandGroup(
            new MoveElevatorAndArm(elevatorTuckPos, armTuckPos),
            drivetrain.followPathCommand(false, pathList.get(1), pTheta2)
          ),
          new ParallelDeadlineGroup(
            // new PlaceCubeHighAuto(drivetrain),
            new RunIntake(0.4).withTimeout(0.2),
            new Drive(0, 0, 0, true)
          ),
          drivetrain.followPathCommand(false, pathList.get(2)),
          new Drive(-5, 0, 0, true).withTimeout(2),
          new AutoBalance()
        )
      ),
      new BoxWheels()
    );
  }
}