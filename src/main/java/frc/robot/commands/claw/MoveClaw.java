// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.claw;

import static frc.robot.Constants.MotionMagicConstants.*;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Claw;

public class MoveClaw extends CommandBase {
  Claw m_claw; 
  double m_speed;
  /** Creates a new setClawPosition. */
  public MoveClaw(Claw claw, double speed) {
    m_claw = claw;
    m_speed = speed; 
    addRequirements(claw);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetPos = m_claw.m_targetPos + m_speed;
    if (targetPos > armStartPos) {
      targetPos = armStartPos;
    } else if (targetPos < armMaxPos) {
      targetPos = armMaxPos;
    }
    // System.out.println(targetPos);
    m_claw.setClawPosition(targetPos);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
