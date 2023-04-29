// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunIntake;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.drivetrain.changeNeutralMode;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCubeHighFromPrePlaceTeleOp extends SequentialCommandGroup {
  /** Creates a new AutoPlaceHigh. */
  public PlaceCubeHighFromPrePlaceTeleOp(Elevator elevator, Arm arm, Intake intake, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new changeNeutralMode(drivetrain, NeutralMode.Brake),
      new SetArmPosition(arm, armHighPlaceCubePos),
      new MoveElevator(elevator, elevatorCubeUpPos),
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new RunIntake(intake, 0.5)
      ),
      new MoveElevatorAndArmFast(elevator, arm, elevatorTuckPos, armTuckPos),
      new changeNeutralMode(drivetrain, NeutralMode.Coast)
    );
  }
}
