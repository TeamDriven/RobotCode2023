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
import frc.robot.commands.automation.ZeroElevatorAndArm;
import frc.robot.commands.drivetrain.Drive;

public final class ThreePlaceTopBlue extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public ThreePlaceTopBlue() {
    List<PathPlannerTrajectory> pathList = PathPlanner.loadPathGroup(
      "ThreePlaceTopBlue", 
      new PathConstraints(3, 3), 
      new PathConstraints(3, 3),
      new PathConstraints(3, 3),
      new PathConstraints(3, 3)
    );

    PIDController pTheta1 = new PIDController(2.5, 0, 0);
    PIDController pTheta2 = new PIDController(1.5, 0, 0);
    PIDController pTheta3 = new PIDController(1.0, 0, 0);
    PIDController pTheta4 = new PIDController(2.5, 0, 0);

    addCommands(
      new RunIntake(-1.0).withTimeout(0.1),
      new ZeroElevatorAndArm(),
      new SetArmPosition(armTuckPos),
      // new WaitCommand(0.1),
      // new PlaceConeHighAuto(elevator, arm, intake, drivetrain);
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
        new RunIntake(0.5).withTimeout(0.2),
        new Drive(0, 0, 0, true)
      ),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(false, pathList.get(2), pTheta3),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
            new MoveElevatorAndArmFast(elevatorPickUpCubePos, armCubePickupPos),
            new RunIntake(-1)
          )
        )
      ),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(false, pathList.get(3), pTheta4),
        new MoveElevatorAndArm(elevatorTuckPos, armTuckPos),
        new RunIntake(-0.15)
      ),
      // new readRetroreflectiveTapeSnapshot(limeLight).withTimeout(0.05),
      // new ParallelDeadlineGroup(
      //   new MoveToLimelight(drivetrain, limeLight),
      //   new readRetroreflectiveTape(limeLight)
      // ),
      new RunIntake(0.5).withTimeout(0.2)
    );
  }
}