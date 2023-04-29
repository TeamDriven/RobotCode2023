// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.MotionMagicConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.automation.PlaceConeHighAuto;
import frc.robot.commands.automation.ZeroElevatorAndArm;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.Drive;

public final class PlaceConeLeftBalanceAuto extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public PlaceConeLeftBalanceAuto() {
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(14.75), 
        new SequentialCommandGroup(
          new ZeroElevatorAndArm(),
          new SetArmPosition(armTuckPos),
          new WaitCommand(0.1),
          new PlaceConeHighAuto(),
          new Drive(-7, 0, 0, true).withTimeout(4.25),
          new Drive(0, -1, 0, false).withTimeout(0.5),
          new Drive(7, 0, 0, true).withTimeout(2.6),
          new AutoBalance()
        )
      ),
      new BoxWheels()
    );
  }
}
