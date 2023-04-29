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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.automation.MoveElevatorAndArm;
import frc.robot.commands.automation.MoveElevatorAndArmFast;
import frc.robot.commands.automation.PlaceConeHighAuto;
import frc.robot.commands.automation.PlaceCubeHighAuto;
import frc.robot.commands.automation.ZeroElevatorAndArm;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LimeLight;

public final class TwoPlaceTopBlue extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }
  
  public TwoPlaceTopBlue(Drivetrain drivetrain, Intake intake, Elevator elevator, Arm arm, LimeLight limeLight) {
    List<PathPlannerTrajectory> pathList = PathPlanner.loadPathGroup(
      "TwoPlaceParkTopBlue", 
      new PathConstraints(3, 3), 
      new PathConstraints(3, 3),
      new PathConstraints(3, 3)
    );

    PIDController pTheta1 = new PIDController(2.5, 0, 0);
    PIDController pTheta2 = new PIDController(1.5, 0, 0);
    // PIDController pTheta3 = new PIDController(2.5, 0, 0);

    addCommands(
      new ZeroElevatorAndArm(elevator, arm),
      new SetArmPosition(arm, armTuckPos),
      new WaitCommand(0.1),
      new PlaceConeHighAuto(elevator, arm, intake, drivetrain),
      new ParallelDeadlineGroup(
        drivetrain.followPathCommand(true, pathList.get(0), pTheta1),
        new SequentialCommandGroup(
          new WaitCommand(1),
          new ParallelCommandGroup(
            new MoveElevatorAndArmFast(elevator, arm, elevatorPickUpCubePos, armCubePickupPos),
            new RunIntake(intake, -1)
          )
        )
      ),
      new ParallelCommandGroup(
        new MoveElevatorAndArm(elevator, arm, elevatorTuckPos, armTuckPos),
        drivetrain.followPathCommand(false, pathList.get(1), pTheta2)
      ),
      // new RunTempIntake(intake, 0.4).withTimeout(0.2)
      new ParallelDeadlineGroup(
        new PlaceCubeHighAuto(elevator, arm, intake, drivetrain),
        new Drive(drivetrain, 0, 0, 0, true)
      )
    );
  }
}