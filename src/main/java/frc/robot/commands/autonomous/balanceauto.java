// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Drivetrain;

public final class BalanceAuto extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public BalanceAuto(final Drivetrain m_Drivetrain) {
    addCommands(
      new Drive(m_Drivetrain, -1.5, 0, 0, true).withTimeout(2),
      new AutoBalance(m_Drivetrain)
      // new Drive(m_Drivetrain,0,0,0,false)
    );
  }
}
