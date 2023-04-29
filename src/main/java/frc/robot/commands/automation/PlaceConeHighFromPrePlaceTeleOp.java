// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import static frc.robot.Constants.MotionMagicConstants.*;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.arm.SetArmPosition;
import frc.robot.commands.drivetrain.ChangeNeutralMode;
import frc.robot.commands.elevator.MoveElevator;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PlaceConeHighFromPrePlaceTeleOp extends SequentialCommandGroup {
  /** Creates a new AutoPlaceHigh. */
  public PlaceConeHighFromPrePlaceTeleOp(Elevator elevator, Arm arm, Drivetrain drivetrain) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ChangeNeutralMode(drivetrain, NeutralMode.Brake),
      new MoveElevator(elevator, elevatorConeUpPos),
      new WaitCommand(0.3),
      new SetArmPosition(arm, armHighPlaceConePos),
      new ChangeNeutralMode(drivetrain, NeutralMode.Coast)
    );
  }
}
