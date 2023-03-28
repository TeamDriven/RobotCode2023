// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.automation;

import static frc.robot.Constants.*;
import static frc.robot.Constants.MotionMagicConstants.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.RunTempIntake;
import frc.robot.commands.drivetrain.AutoTurn;
import frc.robot.commands.drivetrain.AutoTurnToSubstation;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PickUpSubstationCone extends ParallelCommandGroup {
  /** Creates a new PickUpSubstationCone. */
  public PickUpSubstationCone(Drivetrain drivetrain, Intake intake, Elevator elevator, Claw claw) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new RunTempIntake(intake, kIntakeSpeed),
      new MoveElevatorAndClawFast(elevator, claw, elevatorSubstationPos, armSubstationPos),
      new AutoTurnToSubstation(drivetrain)
    );
  }
}
