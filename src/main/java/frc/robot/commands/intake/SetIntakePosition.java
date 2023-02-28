// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
import frc.robot.subsystems.OldIntake;



public class SetIntakePosition extends InstantCommand {

  private final OldIntake m_intake;
  boolean m_position;
  
  public SetIntakePosition(OldIntake subsytem, boolean position) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_intake = subsytem;
    m_position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.intakePosition(m_position);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.resetIntakePosition();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
