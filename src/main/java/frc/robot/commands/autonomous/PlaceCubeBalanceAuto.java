// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import static frc.robot.Constants.MotionMagicConstants.*;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.automation.PlaceCubeHighTeleOp;
import frc.robot.commands.automation.ZeroElevatorAndArm;
import frc.robot.commands.drivetrain.AutoBalance;
import frc.robot.commands.drivetrain.BoxWheels;
import frc.robot.commands.drivetrain.Drive;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public final class PlaceCubeBalanceAuto extends SequentialCommandGroup {

  /** Example static factory for an autonomous command. */
  // public static CommandBase exampleAuto(ExampleSubsystem subsystem) {
  //   return Commands.sequence(subsystem.exampleMethodCommand(), new ExampleCommand(subsystem));
  // }

  public PlaceCubeBalanceAuto(final Drivetrain m_Drivetrain, Elevator elevator, Arm arm, Intake intake) {
    addCommands(
      new ParallelDeadlineGroup(
        new WaitCommand(14.75), 
        new SequentialCommandGroup(
          new ZeroElevatorAndArm(elevator, arm),
          new SetArmPosition(arm, armTuckPos),
          new WaitCommand(0.1),
          new PlaceCubeHighTeleOp(elevator, arm, intake, m_Drivetrain),
          new Drive(m_Drivetrain, -3, 0, 0, true).withTimeout(4.5),
          new Drive(m_Drivetrain, 3, 0, 0, true).withTimeout(2.6),
          new AutoBalance(m_Drivetrain)
        )
      ),
      new BoxWheels(m_Drivetrain)
    );
  }
}
