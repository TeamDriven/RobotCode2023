// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import static frc.robot.Constants.MotionMagicConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class SetClawPositionWaitForFinish extends CommandBase {
  Claw m_claw; 
  double m_targetPos;
  double tolerance = armTicksPerDegree * 5;
  /** Creates a new setClawPosition. */
  public SetClawPositionWaitForFinish(Claw claw, double targetPos) {
    m_claw = claw;
    m_targetPos = targetPos; 
    addRequirements(claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_claw.setClawPosition(m_targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(m_claw.getCurrentPosition());
    System.out.println(m_targetPos);
    if (Math.abs(m_claw.getCurrentPosition() - m_targetPos) < tolerance) {
      return true;
    }
    return false;
  }
}
