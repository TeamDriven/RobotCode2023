// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.limelight;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.drivetrain.MoveToLimelightDriveable;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.LimeLight;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToRetroreflectiveTape extends ParallelCommandGroup {
  /** Creates a new MoveTo2DAprilTags. */
  public MoveToRetroreflectiveTape(LimeLight limeLight, Drivetrain drivetrain, double heading) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new readRetroreflectiveTape(limeLight),
      new MoveToLimelightDriveable(drivetrain, limeLight, heading)
    );
  }
}
