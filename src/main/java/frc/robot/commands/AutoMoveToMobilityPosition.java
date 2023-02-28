// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
// import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
// import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.claw.SetClawPosition;
import frc.robot.commands.claw.SetClawPositionWaitForFinish;
// import frc.robot.commands.claw.SetClawPositionWaitForFinish;
// import frc.robot.commands.claw.ToggleGrip;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.commands.elevator.MoveElevatorWaitForFinish;
// import frc.robot.commands.intake.SetIntakePosition;
// import frc.robot.commands.intake.SpinIntake;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;
// import frc.robot.subsystems.Intake;
import frc.robot.subsystems.ClawPneumatics;
// import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoMoveToMobilityPosition extends SequentialCommandGroup {
  /** Creates a new AutoPlaceConeHigh. */
  public AutoMoveToMobilityPosition(Elevator elevator, Claw claw) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClawPositionWaitForFinish(claw, armStartPos),
      new MoveElevatorWaitForFinish(elevator, elevatorTicksPerInches*1)
      
    );


  }
}
