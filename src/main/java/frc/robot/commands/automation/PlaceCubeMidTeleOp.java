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
import frc.robot.commands.arm.SetArmPositionWaitForFinish;
import frc.robot.commands.drivetrain.ChangeNeutralMode;
import frc.robot.commands.elevator.MoveElevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceCubeMidTeleOp extends SequentialCommandGroup {
  /** Creates a new AutoPlaceHigh. */
  public PlaceCubeMidTeleOp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChangeNeutralMode(NeutralMode.Brake),
      new SetArmPositionWaitForFinish(armMidPlaceCubePos),
      new MoveElevator(elevatorCubeMidPos),
      new WaitCommand(0.5),
      new ParallelDeadlineGroup(
        new WaitCommand(0.25), 
        new RunIntake(0.5)
      ),  
      new MoveElevator(elevatorTuckPos),
      new ChangeNeutralMode(NeutralMode.Coast)
    );
  }
}
