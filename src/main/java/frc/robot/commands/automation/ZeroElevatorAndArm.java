// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.arm.AutoResetArmPosition;
import frc.robot.commands.elevator.AutoResetElevatorPosition;
import frc.robot.commands.elevator.RunElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ZeroElevatorAndArm extends SequentialCommandGroup {
  /** Creates a new ZeroElevatorAndArm. */
  public ZeroElevatorAndArm() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AutoResetElevatorPosition(),
      new ParallelDeadlineGroup(
        new AutoResetArmPosition(), 
        new RunElevator(-0.6)
      ),
      new RunElevator(0.0)
    );
  }
}
