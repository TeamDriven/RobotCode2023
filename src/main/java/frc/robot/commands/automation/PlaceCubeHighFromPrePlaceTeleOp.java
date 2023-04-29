// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.SetClawPositionWaitForFinish;
import frc.robot.commands.drivetrain.changeNeutralMode;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCubeHighFromPrePlaceTeleOp extends SequentialCommandGroup {
  /** Creates a new AutoPlaceHigh. */
  public PlaceCubeHighFromPrePlaceTeleOp(Elevator elevator, Claw claw, Intake intake, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new changeNeutralMode(drivetrain, NeutralMode.Brake),
      new SetClawPosition(claw, armHighPlaceCubePos),
      new MoveElevator(elevator, elevatorCubeUpPos),
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new RunTempIntake(intake, 0.5)
      ),
      new MoveElevatorAndClawFast(elevator, claw, elevatorTuckPos, armTuckPos),
      new changeNeutralMode(drivetrain, NeutralMode.Coast)
    );
  }
}
