// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.automation.PlaceConeHighAuto;
import frc.robot.commands.automation.ZeroElevatorAndClaw;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class PlaceConeBalanceAuto extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public PlaceConeBalanceAuto(final Drivetrain m_Drivetrain, Elevator elevator, Claw claw, Intake intake) {
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(14.75), 
        new SequentialCommandGroup(
          new ZeroElevatorAndClaw(elevator, claw),
          new PlaceConeHighAuto(elevator, claw, intake, m_Drivetrain),
          new Drive(m_Drivetrain, -3, 0, 0, true).withTimeout(5),
          new Drive(m_Drivetrain, 3, 0, 0, true).withTimeout(2.5),
          new AutoBalance(m_Drivetrain)
        )
      ),
      new BoxWheels(m_Drivetrain)
    );
  }
}
