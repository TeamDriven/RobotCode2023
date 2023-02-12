// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.claw.SetClawPnuematics;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.ToggleGrip;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.intake.SetIntakePosition;
import frc.robot.commands.intake.SpinIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ClawPneumatics;
import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoPlaceConeHigh extends SequentialCommandGroup {
  /** Creates a new AutoPlaceConeHigh. */
  public AutoPlaceConeHigh(Intake intake, Elevator elevator, Claw claw, ClawPneumatics clawPneumatics) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new SetIntakePosition(intake, true)
      new SetClawPnuematics(clawPneumatics, true),
      new ParallelDeadlineGroup(
                new WaitCommand(0.25),
                new SetIntakePosition(intake, true),
                new SpinIntake(intake, -0.5)
            ),
      new SetClawPosition(claw, armStartPos),
      new MoveElevator(elevator, elevatorUpPos),
      new WaitCommand(0.5),
      new SetIntakePosition(intake, false)
    );


  }
}
