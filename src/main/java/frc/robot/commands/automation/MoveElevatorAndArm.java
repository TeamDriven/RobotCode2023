// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.SetArmPositionWaitForFinish;
import frc.robot.commands.elevator.MoveElevatorWaitForFinish;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;
import static frc.robot.Constants.MotionMagicConstants.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveElevatorAndArm extends SequentialCommandGroup {
  /** Creates a new AutoPlaceConeHigh. */
  public MoveElevatorAndArm(Elevator elevator, Arm arm, double elevatorPos, double armPos) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetArmPositionWaitForFinish(arm, armTuckPos), //was arm up pos
      new MoveElevatorWaitForFinish(elevator, elevatorPos),
      new SetArmPositionWaitForFinish(arm, armPos)
      // new ParallelDeadlineGroup(
      //    new WaitCommand(1), 
      //    new RunTempIntake(intake, -0.5)
      //  )
    );


  }
}
